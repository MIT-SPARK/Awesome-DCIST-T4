# ZDOTDIR for generated tmux panes (see generate_configs.py pane_shell): the
# pane runs its command, then `exec zsh` with ZDOTDIR here so .zshrc below can
# put the command in the new shell's history. Load the user's real .zshenv.
[[ -r ${_ADT4_ZDOTDIR:-$HOME}/.zshenv ]] && source ${_ADT4_ZDOTDIR:-$HOME}/.zshenv
