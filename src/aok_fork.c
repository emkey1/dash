/*
 * aok_fork.c -- fork by re-launch, for dash compiled as a native program in
 * iSH-AOK.
 *
 * WHY THIS EXISTS
 *
 * A native program in AOK is a C function running on a guest task's thread
 * inside the app's ONE address space, not a process. fork() needs two threads
 * of execution to see DIFFERENT memory at the SAME addresses, which is what a
 * process boundary provides and a thread boundary cannot -- so nlibc_fork
 * refuses with ENOSYS and every one of dash's five forkshell() sites failed.
 * `a | b`, `$(cmd)`, `( cd /tmp )`, `cmd &` and every external command that is
 * not the last thing the shell will ever do all go through them.
 *
 * bash and zsh answered this first (deps/bash/aok_fork.c,
 * deps/zsh/Src/aok_fork.c): spawn a fresh native shell as a new guest task,
 * hand it the parent's state, and have it run the command. A subshell is
 * one-way by definition -- everything it does to its own state is MEANT to be
 * discarded -- so "state in, status and output out" is the actual contract
 * rather than an approximation of fork.
 *
 * WHAT IS DIFFERENT HERE, AND BETTER
 *
 * bash and zsh hand the child a SCRIPT: the state as shell commands, and the
 * command as text. That is forced on them -- bash's state is enormous and its
 * subshell sites are scattered through a recursive evaluator -- and it costs
 * them a re-parse, which is where "a re-launch re-parses the text" and its
 * runaway recursions came from.
 *
 * dash needs none of that, for two reasons:
 *
 *   - Its state is small and completely enumerable: variables, functions,
 *     aliases, options, traps and the positional parameters. Nothing else. So
 *     it travels as a structure, copied field by field, and cannot be
 *     mis-quoted.
 *
 *   - Its five fork sites all hand the child a `union node *` -- the parse
 *     tree of what to run -- and dash already knows how to copy a node tree
 *     into one flat heap block, because that is how it stores a function
 *     definition (copyfunc/calcsize/copynode in nodes.c). So the COMMAND
 *     travels as a tree, not as text, and there is no re-parse at all.
 *
 * The alternative was tried and rejected: jobs.c's commandtext() looks like a
 * way to turn a node back into text, but its own comment says it is for
 * printing in `jobs` output, and cmdputs() handles CTLESC by dropping it
 * (`case CTLESC: c = *p++; break;`). Quoting does not survive, so `echo "a  b"`
 * would come back as two arguments. Text round-trip is unsafe for dash.
 *
 * WHY A POINTER CAN CROSS AT ALL
 *
 * Because parent and child are threads of one process, so a heap address means
 * the same thing in both. That is the same fact that made fork() impossible,
 * used the other way round. It is only safe because dash's globals are now
 * __thread (tools/dash-tls-rewrite.py) -- before that, the child evaluating a
 * tree wrote the PARENT's variable table.
 *
 * The tree is copyfunc'd rather than passed as-is: the parent's own tree lives
 * in the stack allocator and is popped when the command finishes, which for a
 * background job is immediately.
 *
 * WHAT DOES NOT LAUNCH A SHELL AT ALL
 *
 * Two of the five sites never needed a dash on the other end:
 *
 *   - eval.c's vforkexec is a fork whose only purpose is to be overwritten by
 *     an exec. That IS posix_spawn, so aok_vforkexec spawns the command
 *     itself. This is the common case in any script -- one per external
 *     command -- and it costs no shell startup.
 *
 *   - redir.c's openhere forks a writer for a here-document larger than the
 *     pipe buffer. aok_here_fd hands back a descriptor already holding the
 *     text instead, so there is nothing to run concurrently with the reader.
 *
 * WHAT IS NOT HERE
 *
 * Nothing. All five sites are covered. The one thing a re-launch cannot
 * express -- a child that CARRIES ON with the parent's half-executed C stack
 * rather than discarding its state -- does not occur in dash, which is the
 * dividend of dash being a small shell.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <paths.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shell.h"
#include "aok_fork.h"
#include "error.h"
#include "eval.h"
#include "exec.h"
#include "expand.h"
#include "init.h"
#include "jobs.h"
#include "main.h"
#include "memalloc.h"
#include "mystring.h"
#include "nodes.h"
#include "options.h"
#include "output.h"
#include "redir.h"
#include "trap.h"
#include "var.h"
#include "system.h"

/* The re-launched child. Synthesized by the kernel (kernel/native.c), so it
 * exists in every build that compiles this file. There is deliberately no
 * fallback to the guest's own /bin/dash: an emulated child would reject
 * --aok-fork as a bad option, which is a loud failure, where silently running
 * the command with none of the parent's state would not be. */
#define AOK_DASH_PATH "/AOK/native/dash"

/* argv[1] of a re-launched child. Not an environment variable, so that an
 * ordinary program the child runs cannot inherit it, and so that the child
 * does not have to unset it out of its own variable table as well. */
#define AOK_FORK_FLAG "--aok-fork"

/*
 * What the parent hands the child.
 *
 * Reached by token rather than by address: a pointer through argv would be
 * unrecoverable if the child never started, and a token lets a stale or
 * duplicated one be REPORTED rather than dereferenced.
 */
struct aok_req {
	struct aok_req *next;
	unsigned long token;

	/* the command */
	struct funcnode *tree;
	int run;			/* AOK_RUN_* */
	int evalflags;
	char **expfnames;		/* expredir's results, see below */
	int nexpfnames;

	/* what forkchild would have done that posix_spawn cannot */
	int mode;			/* FORK_FG / FORK_BG / FORK_NOJOB */

	/* the state */
	struct aok_vars *vars;
	struct aok_aliases *aliases;
	struct aok_funcs *funcs;
	char *ignored;			/* NSIG bytes: signal is SIG_IGN */
	char opts[NOPTS];
	char **params;			/* positional, NULL-terminated */
	char *arg0;
	int sh_optind, optoff;
	int exitstatus, backgndpid, rootpid, shlvl, lineno, funcline;
};

/*
 * The crossing point, and the ONLY state in dash that is deliberately shared
 * between shells. Everything else is __thread; these three are how a parent
 * hands anything to a child at all, so they are listed in the ALLOW set of
 * tools/bash-tls-fix-externs.py with this note.
 */
static pthread_mutex_t aok_fork_lock = PTHREAD_MUTEX_INITIALIZER;
static struct aok_req *aok_fork_reqs;
static unsigned long aok_fork_next_token = 1;

/* The request this dash was launched to run, if it is a re-launched child.
 * __thread, because every other dash on every other task has its own. */
static __thread struct aok_req *aok_fork_mine;

static void
req_put(struct aok_req *r)
{
	pthread_mutex_lock(&aok_fork_lock);
	r->token = aok_fork_next_token++;
	r->next = aok_fork_reqs;
	aok_fork_reqs = r;
	pthread_mutex_unlock(&aok_fork_lock);
}

/* Take it out of the table. The child does this the moment it finds its own,
 * so a second dash cannot be handed the same one and the list stays short. */
static struct aok_req *
req_take(unsigned long token)
{
	struct aok_req *r, **pp;

	pthread_mutex_lock(&aok_fork_lock);
	for (pp = &aok_fork_reqs; (r = *pp); pp = &r->next) {
		if (r->token == token) {
			*pp = r->next;
			break;
		}
	}
	pthread_mutex_unlock(&aok_fork_lock);
	return r;
}

static void
req_free(struct aok_req *r)
{
	int i;

	if (!r)
		return;
	if (r->tree)
		freefunc(r->tree);
	if (r->vars)
		aok_vars_free(r->vars);
	if (r->aliases)
		aok_aliases_free(r->aliases);
	if (r->funcs)
		aok_funcs_free(r->funcs);
	if (r->params) {
		for (i = 0; r->params[i]; i++)
			ckfree(r->params[i]);
		ckfree(r->params);
	}
	if (r->expfnames) {
		for (i = 0; i < r->nexpfnames; i++)
			if (r->expfnames[i])
				ckfree(r->expfnames[i]);
		ckfree(r->expfnames);
	}
	if (r->arg0)
		ckfree(r->arg0);
	if (r->ignored)
		ckfree(r->ignored);
	ckfree(r);
}

/*
 * expredir's results, which copyfunc does NOT carry.
 *
 * nodetypes marks nfile.expfname `temp`, so mknodes leaves it out of copynode
 * and the copy's field is uninitialised. evalsubshell calls expredir BEFORE
 * forking -- deliberately, so that `( : ) > $(date)` runs date once -- so the
 * expansion has already happened and must travel rather than be redone.
 *
 * They travel as a parallel list because the copy has the same shape as the
 * original, walked in the same order.
 */
static int
count_redirs(union node *n)
{
	int i = 0;

	for (; n; n = n->nfile.next)
		i++;
	return i;
}

static void
save_expfnames(struct aok_req *r, union node *redir)
{
	union node *n;
	int i = 0;

	r->nexpfnames = count_redirs(redir);
	if (!r->nexpfnames)
		return;
	r->expfnames = ckmalloc(r->nexpfnames * sizeof(*r->expfnames));
	for (n = redir; n; n = n->nfile.next, i++) {
		switch (n->type) {
		case NTO: case NCLOBBER: case NFROM:
		case NFROMTO: case NAPPEND:
			r->expfnames[i] = n->nfile.expfname ?
				savestr(n->nfile.expfname) : NULL;
			break;
		default:
			r->expfnames[i] = NULL;
		}
	}
}

static void
load_expfnames(struct aok_req *r, union node *redir)
{
	union node *n;
	int i = 0;

	for (n = redir; n && i < r->nexpfnames; n = n->nfile.next, i++) {
		switch (n->type) {
		case NTO: case NCLOBBER: case NFROM:
		case NFROMTO: case NAPPEND:
			/* The child owns it for as long as it runs, which is
			 * until _exit -- shorter than the request. */
			n->nfile.expfname = r->expfnames[i];
			break;
		}
	}
}

/* ---------------------------------------------------------- the parent side */

static char **
save_params(void)
{
	char **p;
	int i;

	for (i = 0; shellparam.p && shellparam.p[i]; i++)
		;
	p = ckmalloc((i + 1) * sizeof(*p));
	for (i = 0; shellparam.p && shellparam.p[i]; i++)
		p[i] = savestr(shellparam.p[i]);
	p[i] = NULL;
	return p;
}

static struct aok_req *
build_req(union node *n, int run, int evalflags, int mode)
{
	struct aok_req *r;
	int i;

	r = ckmalloc(sizeof(*r));
	memset(r, 0, sizeof(*r));
	r->run = run;
	r->evalflags = evalflags;
	r->mode = mode;
	if (n) {
		r->tree = copyfunc(n);
		if (run == AOK_RUN_SUBSHELL)
			save_expfnames(r, n->nredir.redirect);
	}

	r->vars = aok_vars_save();
	r->aliases = aok_aliases_save();
	r->funcs = aok_funcs_save();

	/* Traps do NOT travel, apart from the ignored ones. jobs.c's
	 * FORKRESET clears every trap that has a command and keeps the ones
	 * set to SIG_IGN (`trap "" INT`), which is what POSIX says a subshell
	 * gets, so the child is given exactly that much. */
	r->ignored = ckmalloc(NSIG);
	aok_traps_save(r->ignored);

	memcpy(r->opts, optlist, sizeof(r->opts));
	r->params = save_params();
	r->arg0 = arg0 ? savestr(arg0) : NULL;
	r->sh_optind = shellparam.sh_optind;
	r->optoff = shellparam.optoff;

	r->exitstatus = exitstatus;
	r->backgndpid = backgndpid;
	/* $$ is the ROOT shell's pid and POSIX requires a subshell to report
	 * the same one, so this is inherited rather than re-read. */
	r->rootpid = rootpid;
	r->shlvl = shlvl + 1;		/* forkchild's shlvl++ */
	r->lineno = lineno;
	r->funcline = aok_funcline_get();
	return r;
}

/*
 * Spawn the child. `pgid` is the process group to put it in, or -1 for none.
 */
static int
spawn_child(struct aok_req *r, const struct aok_fork_io *io, int pgid)
{
	char token[32];
	char *argv[4];
	void *fa = NULL, *attr = NULL;
	pid_t pid = -1;
	int err, i;

	if (posix_spawn_file_actions_init(&fa) != 0)
		return -1;

	if (io) {
		if (io->devnull_in)
			posix_spawn_file_actions_addopen(&fa, 0, _PATH_DEVNULL,
							 O_RDONLY, 0);
		if (io->in_fd >= 0) {
			posix_spawn_file_actions_adddup2(&fa, io->in_fd, 0);
			if (io->in_fd != 0)
				posix_spawn_file_actions_addclose(&fa,
								  io->in_fd);
		}
		if (io->out_fd >= 0) {
			posix_spawn_file_actions_adddup2(&fa, io->out_fd, 1);
			if (io->out_fd != 1)
				posix_spawn_file_actions_addclose(&fa,
								  io->out_fd);
		}
		for (i = 0; i < io->nclose; i++)
			if (io->close_fds[i] >= 0)
				posix_spawn_file_actions_addclose(&fa,
							io->close_fds[i]);
	}

	if (pgid >= 0 && posix_spawnattr_init(&attr) == 0) {
		posix_spawnattr_setpgroup(&attr, pgid);
		posix_spawnattr_setflags(&attr, NLIBC_SPAWN_SETPGROUP);
	} else {
		attr = NULL;
	}

	fmtstr(token, sizeof(token), "%lu", r->token);
	argv[0] = arg0 ? arg0 : (char *)"dash";
	argv[1] = (char *)AOK_FORK_FLAG;
	argv[2] = token;
	argv[3] = NULL;

	err = posix_spawn(&pid, AOK_DASH_PATH, &fa, attr ? &attr : NULL,
			  argv, environment());
	if (attr)
		posix_spawnattr_destroy(&attr);
	posix_spawn_file_actions_destroy(&fa);
	if (err != 0) {
		errno = err;
		return -1;
	}
	return pid;
}

int
aok_forkshell(struct job *jp, union node *n, int mode, int run, int flags,
	const struct aok_fork_io *io)
{
	struct aok_fork_io local;
	struct aok_req *r;
	int pgid = -1;
	int pid;

	if (!io) {
		memset(&local, 0, sizeof(local));
		local.in_fd = local.out_fd = -1;
		io = &local;
	}

	r = build_req(n, run, flags, mode);

	/* forkchild's job-control half. The pgid has to be decided here
	 * because posix_spawn sets it before the child runs -- and because
	 * upstream sets it in BOTH parent and child on purpose, so that
	 * neither ordering loses the race. */
#if JOBS
	if (mode != FORK_NOJOB && jp && jp->jobctl && !shlvl)
		pgid = jp->nprocs == 0 ? 0 : jp->ps[0].pid;
#endif
	/* And the half posix_spawn cannot express: a backgrounded job with no
	 * job control reads /dev/null rather than competing with the
	 * interactive shell for keystrokes. */
	if (mode == FORK_BG && pgid < 0 && jp && jp->nprocs == 0 &&
	    !io->devnull_in && io->in_fd < 0) {
		local = *io;
		local.devnull_in = 1;
		io = &local;
	}

	req_put(r);
	pid = spawn_child(r, io, pgid);
	if (pid < 0) {
		req_take(r->token);
		req_free(r);
		return -1;
	}
#if JOBS
	if (mode == FORK_FG && pgid >= 0)
		aok_tcsetpgrp_fg(pgid ? pgid : pid);
#endif
	return pid;
}

/* ------------------------------------------------- external commands (exec) */

/*
 * dash's tryexec, as a spawn.
 *
 * The ENOEXEC arm is upstream's and matters more than it looks: a file with no
 * #! and no recognisable header is run as a shell script, which is how a great
 * many system scripts still start.
 */
static int
try_spawn(const char *cmd, char **argv, char **envp, void *fa, void *attr,
	pid_t *pid)
{
	char **ap, **new;
	int err, i;

	err = posix_spawn(pid, cmd, &fa, attr ? &attr : NULL, argv, envp);
	if (err != ENOEXEC)
		return err;

	for (ap = argv; *ap; ap++)
		;
	new = stalloc((ap - argv + 2) * sizeof(char *));
	new[0] = (char *)"sh";
	new[1] = (char *)cmd;
	for (i = 1; argv[i]; i++)
		new[i + 1] = argv[i];
	new[i + 1] = NULL;
	return posix_spawn(pid, _PATH_BSHELL, &fa, attr ? &attr : NULL,
			   new, envp);
}

struct job *
aok_vforkexec(union node *n, char **argv, const char *path, int idx)
{
	struct job *jp;
	char **envp;
	char *cmdname;
	void *fa = NULL, *attr = NULL;
	pid_t pid = -1;
	int e, exerrno, pgid = -1;

	/*
	 * The job is made AFTER the spawn, not before. Upstream has to make it
	 * first because fork() needs somewhere to record the child before the
	 * child exists; here the pid is known before there is anything to
	 * record, and doing it in this order means the failure path has no job
	 * to tear down -- which is the whole reason freejob would have had to
	 * stop being a file-static.
	 *
	 * Nothing is lost by it: a fresh one-process job's pgid is its own
	 * (0 to posix_spawn), and its jobctl comes from the global.
	 */
	envp = environment();
	if (posix_spawn_file_actions_init(&fa) != 0) {
		e = errno;
		goto fail;
	}
#if JOBS
	if (jobctl && !shlvl) {
		pgid = 0;
		if (posix_spawnattr_init(&attr) == 0) {
			posix_spawnattr_setpgroup(&attr, pgid);
			posix_spawnattr_setflags(&attr, NLIBC_SPAWN_SETPGROUP);
		} else {
			attr = NULL;
		}
	}
#endif

	if (strchr(argv[0], '/') != NULL) {
		e = try_spawn(argv[0], argv, envp, fa, attr, &pid);
	} else {
		const char *p = path;

		e = ENOENT;
		while (padvance(&p, argv[0]) >= 0) {
			cmdname = stackblock();
			if (--idx < 0 && pathopt == NULL) {
				int r = try_spawn(cmdname, argv, envp, fa,
						  attr, &pid);
				if (r == 0) {
					e = 0;
					break;
				}
				if (r != ENOENT && r != ENOTDIR)
					e = r;
			}
		}
	}

	if (attr)
		posix_spawnattr_destroy(&attr);
	posix_spawn_file_actions_destroy(&fa);
	fa = NULL;
	attr = NULL;

	if (e == 0) {
#if JOBS
		if (pgid >= 0)
			aok_tcsetpgrp_fg(pid);
#endif
		jp = makejob(n, 1);
		forkparent(jp, n, FORK_FG, pid);
		return jp;
	}

fail:
	if (attr)
		posix_spawnattr_destroy(&attr);
	if (fa)
		posix_spawn_file_actions_destroy(&fa);
	/* Upstream's mapping, from shellexec. It happens in the CHILD there,
	 * so it exits with the status; here the shell is still standing and
	 * has to set it. */
	switch (e) {
	default:
		exerrno = 126;
		break;
	case ELOOP:
	case ENAMETOOLONG:
	case ENOENT:
	case ENOTDIR:
		exerrno = 127;
		break;
	}
	exitstatus = exerrno;
	/* A WARNING, not an exerror. Upstream this message is printed by the
	 * child, from shellexec, and the child then exits -- the shell itself
	 * carries on and just sees the status. Raising here would unwind the
	 * shell's own evaluation instead, which upstream never does at this
	 * point. waitforjob(NULL) returns exitstatus, so the caller reads the
	 * 126/127 unchanged. */
	sh_warnx("%s: %s", argv[0], errmsg(e, E_EXEC));
	return NULL;
}

/* ----------------------------------------------------------- here-documents */

int
aok_here_fd(const char *p, size_t len)
{
	/* TMPDIR first because a user can point it somewhere writable, then
	 * the two places POSIX says are there. A here-document bigger than
	 * the pipe buffer on a system with none of the three is the one case
	 * this cannot serve, and openhere says so rather than truncating. */
	static const char *const dirs[] = { NULL, "/tmp", "/var/tmp" };
	char name[PATH_MAX];
	const char *dir;
	unsigned i;
	int fd = -1;

	for (i = 0; i < sizeof(dirs) / sizeof(*dirs); i++) {
		dir = dirs[i] ? dirs[i] : lookupvar("TMPDIR");
		if (!dir || !*dir)
			continue;
		if ((size_t) fmtstr(name, sizeof(name), "%s/dash-here-XXXXXX",
				    dir) >= sizeof(name))
			continue;
		fd = mkstemp(name);
		if (fd >= 0)
			break;
	}
	if (fd < 0)
		return -1;
	/* Unlinked immediately: nothing else ever needs the name, and there is
	 * then no window in which a path exists to be raced or left behind. */
	unlink(name);
	while (len) {
		ssize_t w = write(fd, p, len);

		if (w < 0) {
			if (errno == EINTR)
				continue;
			close(fd);
			return -1;
		}
		p += w;
		len -= w;
	}
	if (lseek(fd, 0, SEEK_SET) < 0) {
		close(fd);
		return -1;
	}
	return fd;
}

/* ----------------------------------------------------------- the child side */

static void
child_apply_state(struct aok_req *r)
{
	int i;

	/* Options FIRST: -e, -u and -x change what everything below does, and
	 * -f changes how a pattern in a variable's value would be read. */
	memcpy(optlist, r->opts, sizeof(optlist));

	aok_vars_load(r->vars);
	aok_aliases_load(r->aliases);
	aok_funcs_load(r->funcs);

	aok_traps_load(r->ignored);

	if (r->arg0)
		arg0 = savestr(r->arg0);
	shellparam.p = r->params;
	r->params = NULL;		/* the shell owns them now */
	shellparam.malloc = 1;
	for (i = 0; shellparam.p[i]; i++)
		;
	shellparam.nparam = i;
	shellparam.sh_optind = r->sh_optind;
	shellparam.optoff = r->optoff;

	exitstatus = r->exitstatus;
	backgndpid = r->backgndpid;
	rootpid = r->rootpid;
	shlvl = r->shlvl;
	lineno = r->lineno;
	aok_funcline_set(r->funcline);

	/* The rest of forkchild that posix_spawn could not do for us. */
#if JOBS
	jobctl = 0;
#endif
	if (r->mode == FORK_BG) {
		ignoresig(SIGINT);
		ignoresig(SIGQUIT);
	}
}

int
aok_fork_child(int argc, char **argv)
{
	struct aok_req *r;
	union node *n;
	char *end;
	unsigned long token;

	if (argc != 3 || strcmp(argv[1], AOK_FORK_FLAG) != 0)
		return 0;

	token = strtoul(argv[2], &end, 10);
	r = (end && !*end) ? req_take(token) : NULL;
	if (!r) {
		/* Not survivable and not silent: this dash was launched to run
		 * something and no longer knows what. */
		outfmt(out2, "dash: %s %s: no such handoff\n",
		       AOK_FORK_FLAG, argv[2]);
		exitstatus = 2;
		exitshell();
	}
	aok_fork_mine = r;

	child_apply_state(r);

	n = r->tree ? &r->tree->n : NULL;
	if (n && r->run == AOK_RUN_SUBSHELL) {
		load_expfnames(r, n->nredir.redirect);
		redirect(n->nredir.redirect, 0);
		n = n->nredir.n;
	}
	if (n)
		evaltree(n, r->evalflags);
	exitshell();
	/* NOTREACHED */
	return 1;
}

void
aok_fork_child_release(void)
{
	struct aok_req *r = aok_fork_mine;

	if (!r)
		return;
	aok_fork_mine = NULL;
	/* load_expfnames aliased these strings into nodes of the tree. Nothing
	 * frees them from that side -- freefunc releases the tree as one block
	 * and never looks at expfname, which is why nodetypes marks it `temp`
	 * -- so req_free below is the only free, not a second one. */
	req_free(r);
}
