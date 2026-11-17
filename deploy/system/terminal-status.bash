# BEGIN terminal status
case $- in
  *i*)
    if [ -r "$HOME/wonderduck/scripts/terminal-status.py" ]; then
      python3 "$HOME/wonderduck/scripts/terminal-status.py"
    fi
    ;;
esac
# END terminal status
