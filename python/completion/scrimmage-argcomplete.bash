# GNU Lesser General Public License v3.0

if type register-python-argcomplete3 > /dev/null 2>&1; then
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 scrimpy)"
elif type register-python-argcomplete > /dev/null 2>&1; then
  eval "$(register-python-argcomplete ros2)"
  eval "$(register-python-argcomplete scrimpy)"
fi
