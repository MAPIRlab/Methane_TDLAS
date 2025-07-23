from ros2run.api import get_executable_path
from ros2launch.api import get_share_file_path_from_package
import sys
from getpass import getpass
from subprocess import Popen, PIPE

exec_path = get_executable_path(package_name="minirae_lite", executable_name="miniraeLiteID")
rule_path = get_share_file_path_from_package(package_name="minirae_lite", file_name="90-minirae_lite.rules")

password = getpass("Please enter your password: ")

# sudo requires the flag '-S' in order to take input from stdin
copyExec = Popen(f"sudo -S cp {exec_path} /usr/bin/miniraeLiteID".split(), stdin=PIPE, stdout=PIPE, stderr=PIPE)
copyExec.communicate(password.encode())


copyRule = Popen(f"sudo -S cp {rule_path} /etc/udev/rules.d/90-minirae_lite.rules".split(), stdin=PIPE, stdout=PIPE, stderr=PIPE)
copyRule.communicate(password.encode())
