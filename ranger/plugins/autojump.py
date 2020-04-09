import ranger.api
import subprocess
from ranger.api.commands import *

HOOK_INIT_OLD = ranger.api.hook_init


def hook_init(fm):
    def update_autojump(signal):
        subprocess.Popen(["autojump", "--add", signal.new.path])

    fm.signal_bind('cd', update_autojump)
    HOOK_INIT_OLD(fm)


ranger.api.hook_init = hook_init


class j(Command):
    """:j

    Uses autojump to set the current directory.
    """

    def execute(self):
        directory = subprocess.check_output(["autojump", self.arg(1)])
        directory = directory.decode("utf-8", "ignore")
        directory = directory.rstrip('\n')
        self.fm.execute_console("cd " + directory)

    def tab(self, tabnum):
        # This is a generic tab-completion function that iterates through the
        completion_directories = subprocess.check_output(["autojump", "--complete", self.arg(1)]).split('\n')
        return ["j " + candidate for candidate in completion_directories if candidate != '']
