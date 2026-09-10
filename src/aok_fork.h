/*
 * aok_fork.h -- fork by re-launch, for dash compiled as a native program in
 * iSH-AOK. The design is in aok_fork.c; this is the seam the rest of dash
 * calls through.
 */

#ifndef AOK_FORK_H
#define AOK_FORK_H

struct job;
union node;
struct funcnode;

/*
 * The descriptor work the child needs done before it evaluates anything.
 *
 * A real forkchild does this itself, between fork() and the evaluation,
 * because it is already the child. A re-launched child is not yet running when
 * the decision is made, so the parent hands the list to posix_spawn as file
 * actions and the child inherits the result. Same effect, opposite side of the
 * boundary.
 *
 * in_fd/out_fd are dup2'd onto 0 and 1 and then closed; -1 means "leave it".
 * devnull_in opens /dev/null on 0 instead, which is what a background job with
 * no job control gets (jobs.c's forkchild).
 */
struct aok_fork_io {
	int in_fd;
	int out_fd;
	int devnull_in;
	int close_fds[4];
	int nclose;
};

/* What the child does with the node it is handed. */
#define AOK_RUN_TREE     0	/* evaltree(n, flags) */
#define AOK_RUN_SUBSHELL 1	/* redirect(n->nredir.redirect, 0), then
				 * evaltree(n->nredir.n, flags) */

/*
 * The parent half. Spawns a fresh native dash on a new guest task, hands it
 * this shell's state and `n`, and returns the child's pid -- or -1 with errno
 * set, which the caller turns into "Cannot fork" through forkparent().
 *
 * Never returns 0: unlike fork(), there is no arm of this that is the child.
 */
int aok_forkshell(struct job *jp, union node *n, int mode, int run, int flags,
	const struct aok_fork_io *io);

/*
 * The external-command path -- eval.c's vforkexec. No dash is launched at all;
 * the command itself is spawned, because a fork whose only purpose is to be
 * overwritten by an exec is exactly what posix_spawn already is.
 */
struct job *aok_vforkexec(union node *n, char **argv, const char *path,
	int idx);

/*
 * A here-document too big for the pipe buffer. Upstream forks a writer; this
 * returns a descriptor already holding the text instead, so nothing has to run
 * concurrently with the reader.
 */
int aok_here_fd(const char *p, size_t len);

/*
 * The child half. Called from main() once the shell is initialised: returns 0
 * if this dash is an ordinary one, and otherwise never returns.
 */
int aok_fork_child(int argc, char **argv);

/* Called from exitshell(), on the one path every dash leaves through. */
void aok_fork_child_release(void);

/*
 * Per-file snapshot and restore. Each lives beside the table it reads, because
 * vartab, atab and cmdtable are all file-statics -- exporting the tables
 * themselves to reach them from here would undo that.
 */
struct aok_vars   *aok_vars_save(void);
void               aok_vars_load(struct aok_vars *);
void               aok_vars_free(struct aok_vars *);
struct aok_aliases *aok_aliases_save(void);
void               aok_aliases_load(struct aok_aliases *);
void               aok_aliases_free(struct aok_aliases *);
struct aok_funcs  *aok_funcs_save(void);
void               aok_funcs_load(struct aok_funcs *);
void               aok_funcs_free(struct aok_funcs *);

/* eval.c's funcline, which is a file-static and is what $LINENO is measured
 * against inside a function. */
int  aok_funcline_get(void);
void aok_funcline_set(int);

#endif /* AOK_FORK_H */
