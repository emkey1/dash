/*-
 * Copyright (c) 2026 iSH-AOK contributors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * aok_mksignames.c -- generate signames.c, replacing dash's mksignames.c.
 *
 * WHY THIS FILE EXISTS
 *
 * dash is BSD-3-Clause with one exception: src/mksignames.c is GPL-2+, taken
 * from bash (its own header says "you should have received a copy of the GNU
 * General Public License along with Bash"). It is a build-time helper and is
 * not linked -- but its OUTPUT is, which Debian's copyright file flags in as
 * many words:
 *
 *     Files: src/mksignames.c
 *     Comment: This file is not directly linked with dash.  However, its
 *              output is.
 *     License: GPL-2+
 *
 * iSH-AOK ships in an App Store build, where GPL of any version is a problem --
 * the FSF's position on Apple's Usage Rules covers all versions, not only v3 --
 * so the GPL file is removed from this tree entirely and this replaces it.
 * Written from the INTERFACE its two callers require, not from the original:
 *
 *     src/trap.c:81   extern char *signal_names[];
 *                     trap with no arguments loops signo = 0 .. NSIG-1 and
 *                     prints signal_names[signo], so index 0 must be "EXIT".
 *     src/jobs.c:251  kill -l prints signal_names[i] for i = 1 .. NSIG-1.
 *
 * Both want the bare name, without the SIG prefix: `trap -- 'cmd' USR1`, and
 * `kill -l` listing HUP, INT, QUIT.
 *
 * This file is licensed as the rest of dash: BSD-3-Clause.
 *
 * HOW IT GETS THE NAMES
 *
 * From the platform's own <signal.h>, through #ifdef, so a signal the host does
 * not have simply is not emitted and one it has that this list does not know
 * still gets a usable name from the numeric fallback below.
 *
 * WHICH NUMBERING THIS TABLE IS INDEXED BY, since iSH-AOK compiles dash as a
 * native program against its own shim and the answer is not obvious. It is the
 * HOST's, and that is correct, because the shim translates at every boundary
 * rather than letting the two numberings meet:
 *
 *   kernel/native_libc.c  nlibc_signal_to_guest()      on the way out
 *   kernel/native_libc.c  nlibc_wait_status_to_host()  on the way back
 *
 * The second exists because the numbers disagree exactly where it hurts: guest
 * SIGUSR1 is 10, which is Darwin's SIGBUS, and before that translation a native
 * bash reported a child killed by SIGUSR1 as "Bus error". So a native program
 * sees host numbering throughout, this table is built from the host's
 * <signal.h>, and the two agree by construction.
 *
 * (An earlier version of this comment claimed the opposite -- that the guest's
 * Linux numbering is what reaches here. It does not, and a table built on that
 * belief would have been wrong for every signal above 15.)
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef NSIG
#ifdef _NSIG
#define NSIG _NSIG
#else
#define NSIG 65
#endif
#endif

/* Working size only. The EMITTED array is `[NSIG + 1]` -- NSIG names starting
 * with EXIT at 0, then a NULL terminator -- which is what dash's own generator
 * produced and what its callers were compiled against. Taking this number from
 * mksignames.c's internal buffer instead (2 * NSIG + 3) produced an array
 * twice the right length; the diff against the real output is what caught it. */
#define SIGNAMES_SIZE (NSIG + 1)

struct signame {
	int signo;
	const char *name;
};

/* Every signal name POSIX defines, plus the common extensions, each guarded so
 * that a platform lacking one drops it rather than failing to compile. Listed
 * by name rather than derived, because there is no portable way to ask
 * <signal.h> for the spelling of a number. */
static const struct signame known[] = {
#ifdef SIGHUP
	{ SIGHUP, "HUP" },
#endif
#ifdef SIGINT
	{ SIGINT, "INT" },
#endif
#ifdef SIGQUIT
	{ SIGQUIT, "QUIT" },
#endif
#ifdef SIGILL
	{ SIGILL, "ILL" },
#endif
#ifdef SIGTRAP
	{ SIGTRAP, "TRAP" },
#endif
#ifdef SIGABRT
	{ SIGABRT, "ABRT" },
#endif
#ifdef SIGIOT
	{ SIGIOT, "IOT" },
#endif
#ifdef SIGBUS
	{ SIGBUS, "BUS" },
#endif
#ifdef SIGFPE
	{ SIGFPE, "FPE" },
#endif
#ifdef SIGKILL
	{ SIGKILL, "KILL" },
#endif
#ifdef SIGUSR1
	{ SIGUSR1, "USR1" },
#endif
#ifdef SIGSEGV
	{ SIGSEGV, "SEGV" },
#endif
#ifdef SIGUSR2
	{ SIGUSR2, "USR2" },
#endif
#ifdef SIGPIPE
	{ SIGPIPE, "PIPE" },
#endif
#ifdef SIGALRM
	{ SIGALRM, "ALRM" },
#endif
#ifdef SIGTERM
	{ SIGTERM, "TERM" },
#endif
#ifdef SIGSTKFLT
	{ SIGSTKFLT, "STKFLT" },
#endif
#ifdef SIGCHLD
	{ SIGCHLD, "CHLD" },
#endif
#ifdef SIGCONT
	{ SIGCONT, "CONT" },
#endif
#ifdef SIGSTOP
	{ SIGSTOP, "STOP" },
#endif
#ifdef SIGTSTP
	{ SIGTSTP, "TSTP" },
#endif
#ifdef SIGTTIN
	{ SIGTTIN, "TTIN" },
#endif
#ifdef SIGTTOU
	{ SIGTTOU, "TTOU" },
#endif
#ifdef SIGURG
	{ SIGURG, "URG" },
#endif
#ifdef SIGXCPU
	{ SIGXCPU, "XCPU" },
#endif
#ifdef SIGXFSZ
	{ SIGXFSZ, "XFSZ" },
#endif
#ifdef SIGVTALRM
	{ SIGVTALRM, "VTALRM" },
#endif
#ifdef SIGPROF
	{ SIGPROF, "PROF" },
#endif
#ifdef SIGWINCH
	{ SIGWINCH, "WINCH" },
#endif
#ifdef SIGIO
	{ SIGIO, "IO" },
#endif
#ifdef SIGPOLL
	{ SIGPOLL, "POLL" },
#endif
#ifdef SIGPWR
	{ SIGPWR, "PWR" },
#endif
#ifdef SIGINFO
	{ SIGINFO, "INFO" },
#endif
#ifdef SIGSYS
	{ SIGSYS, "SYS" },
#endif
#ifdef SIGEMT
	{ SIGEMT, "EMT" },
#endif
};

int
main(int argc, char **argv)
{
	static const char *names[SIGNAMES_SIZE];
	char numeric[SIGNAMES_SIZE][16];
	/* Writes signames.c ITSELF, because that is what the build rule expects:
	 * `signames.c: aok_mksignames` runs `./$^` with no redirection, exactly
	 * as it did for dash's own generator. Writing to stdout instead builds
	 * cleanly, generates nothing, and fails later with a missing file --
	 * which is how this was found. */
	const char *path = (argc > 1) ? argv[1] : "signames.c";
	FILE *out;
	size_t i;
	int signo;

	out = fopen(path, "w");
	if (out == NULL) {
		perror(path);
		return 1;
	}

	for (i = 0; i < SIGNAMES_SIZE; i++)
		names[i] = NULL;

	/* Index 0 is not a signal. trap prints it for the EXIT pseudo-signal,
	 * which is the whole reason this array is offset the way it is. */
	names[0] = "EXIT";

	for (i = 0; i < sizeof(known) / sizeof(known[0]); i++) {
		signo = known[i].signo;
		if (signo > 0 && (size_t) signo < SIGNAMES_SIZE) {
			/* First spelling wins, so an alias (IOT for ABRT, POLL
			 * for IO) does not displace the canonical name that
			 * came before it in the list. */
			if (names[signo] == NULL)
				names[signo] = known[i].name;
		}
	}

	/* Real-time signals are a contiguous range whose numbering is decided at
	 * runtime on some platforms, so they are named by position rather than
	 * listed: RTMIN, RTMIN+1 ... RTMAX-1, RTMAX, which is what every shell
	 * prints for them. */
#if defined(SIGRTMIN) && defined(SIGRTMAX)
	{
		int rtmin = SIGRTMIN;
		int rtmax = SIGRTMAX;
		int half = rtmin + (rtmax - rtmin) / 2;

		for (signo = rtmin; signo <= rtmax && (size_t) signo < SIGNAMES_SIZE;
		     signo++) {
			if (names[signo] != NULL)
				continue;
			if (signo == rtmin)
				snprintf(numeric[signo], sizeof(numeric[0]), "RTMIN");
			else if (signo == rtmax)
				snprintf(numeric[signo], sizeof(numeric[0]), "RTMAX");
			else if (signo <= half)
				snprintf(numeric[signo], sizeof(numeric[0]),
					 "RTMIN+%d", signo - rtmin);
			else
				snprintf(numeric[signo], sizeof(numeric[0]),
					 "RTMAX-%d", rtmax - signo);
			names[signo] = numeric[signo];
		}
	}
#endif

	/* Anything still unnamed BELOW NSIG gets its number. A hole in the
	 * middle of the range is normal -- 32 and 33 are glibc's, and are not
	 * nameable -- and a NULL there is a crash in `kill -l`, which walks
	 * 1..NSIG-1 unconditionally (jobs.c) with no NULL check.
	 *
	 * The tail above NSIG is deliberately left NULL, which is what dash's
	 * own generator produced. Nothing reads it -- both callers stop at
	 * NSIG -- and matching exactly makes the two outputs diffable, which is
	 * how this file was checked: identical for every index dash touches. */
	for (signo = 1; signo < NSIG && (size_t) signo < SIGNAMES_SIZE; signo++) {
		if (names[signo] != NULL)
			continue;
		snprintf(numeric[signo], sizeof(numeric[0]), "%d", signo);
		names[signo] = numeric[signo];
	}

	fputs("/* Generated by aok_mksignames.c -- do not edit.\n"
	      " *\n"
	      " * Replaces dash's own mksignames.c, which is GPL-2+ and is not\n"
	      " * part of this tree. See src/aok_mksignames.c for why.\n"
	      " *\n"
	      " * BSD-3-Clause, as the rest of dash.\n"
	      " */\n\n", out);
	/* NSIG comes from the header rather than from this generator, so the
	 * array is sized by the compiler that builds dash rather than by the
	 * one that built this. The shape, the const-ness and the trailing NULL
	 * are all as dash's own generator emitted them. */
	fputs("#include <signal.h>\n\n", out);
	fputs("/* A translation list so we can be polite to our users. */\n", out);
	fputs("const char *const signal_names[NSIG + 1] = {\n", out);
	for (signo = 0; signo < NSIG; signo++) {
		if (names[signo] == NULL)
			fputs("    (char *)0x0,\n", out);
		else
			fprintf(out, "    \"%s\",\n", names[signo]);
	}
	fputs("    (char *)0x0\n};\n", out);

	if (fclose(out) != 0) {
		perror("fclose");
		return 1;
	}
	return 0;
}
