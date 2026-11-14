# ZDOTDIR for generated tmux panes (see generate_configs.py pane_shell).
# Restore the user's ZDOTDIR, load their .zshrc, then make the pane's command
# recallable with up-arrow (ADT4_PANE_CMD) or, for commands that are only
# prepared and not run (ADT4_PANE_PREFILL), put it on the prompt.
if [[ -n ${_ADT4_ZDOTDIR-} ]]; then
  ZDOTDIR=$_ADT4_ZDOTDIR
else
  unset ZDOTDIR
fi
unset _ADT4_ZDOTDIR
[[ -r ${ZDOTDIR:-$HOME}/.zshrc ]] && source ${ZDOTDIR:-$HOME}/.zshrc
if [[ -n ${ADT4_PANE_CMD-} ]]; then
  if [[ -n ${ADT4_PANE_PREFILL-} ]]; then
    print -rz -- "$ADT4_PANE_CMD"
  else
    # zsh loads HISTFILE after the rc files, so add the entry at the first prompt
    typeset -g _adt4_pane_cmd=$ADT4_PANE_CMD
    _adt4_pane_history() {
      print -rs -- "$_adt4_pane_cmd"
      unset _adt4_pane_cmd
      add-zsh-hook -d precmd _adt4_pane_history
      unfunction _adt4_pane_history
    }
    autoload -Uz add-zsh-hook
    add-zsh-hook precmd _adt4_pane_history
  fi
fi
unset ADT4_PANE_CMD ADT4_PANE_PREFILL
