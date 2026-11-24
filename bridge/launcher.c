// Tiny launcher for OBD Bridge.app: runs Contents/Resources/run.sh as a child so that
// macOS attributes Bluetooth access to this app bundle (which declares it in Info.plist).
// A bare Python interpreter has no bundle, and macOS kills it when it touches Bluetooth.
#include <mach-o/dyld.h>
#include <limits.h>
#include <libgen.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
extern char **environ;
static pid_t child = 0;
static void fwd(int s) { if (child > 0) kill(child, s); }
int main(void) {
    char exe[PATH_MAX], real[PATH_MAX], script[PATH_MAX];
    uint32_t n = sizeof exe;
    if (_NSGetExecutablePath(exe, &n) != 0 || !realpath(exe, real)) return 1;
    snprintf(script, sizeof script, "%s/../Resources/run.sh", dirname(real));
    char *argv[] = {"/bin/zsh", script, NULL};
    signal(SIGTERM, fwd); signal(SIGINT, fwd); signal(SIGHUP, fwd);
    if (posix_spawn(&child, "/bin/zsh", NULL, NULL, argv, environ) != 0) return 2;
    int st = 0; while (waitpid(child, &st, 0) < 0) {}
    return WIFEXITED(st) ? WEXITSTATUS(st) : 3;
}
