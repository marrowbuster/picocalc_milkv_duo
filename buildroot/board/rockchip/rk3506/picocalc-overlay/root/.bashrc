export PS1='\W\$ '
alias ls='ls --color=auto'
alias grep='grep --color=auto'
if [ -z "$SSH_CONNECTION" ]; then
   if [[ "$(tty)" =~ /dev/console ]] then
        return
   elif [[ "$(tty)" =~ /dev/tty ]] && type fbterm > /dev/null 2>&1; then
        fbterm 
  # otherwise, start/attach to tmux and start neofetch
   elif [ -z "$TMUX" ] && type tmux >/dev/null 2>&1; then
        tmux new -As "$(basename $(tty))" 'cd ~/command-launcher; fff; cd ~; bash'
   fi
fi
