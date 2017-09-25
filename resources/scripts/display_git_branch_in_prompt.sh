# display git branch in prompt
function git_branch 
{
   git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1) /'
}

bash_color=32

if [ $(hostname) == "mbot05h" ]; then
   #bash_color=31 # red color
   #bash_color=32 # green color
   #bash_color=33 # yellow color
   bash_color=34 # blue color
   #bash_color=35 # purple color
   #bash_color=36 # cyan color

elif [ $(hostname) == "mbot05n" ]; then
   bash_color=35
elif [ $(hostname) == "harode-server" ]; then
   bash_color=36
fi

#select this one to show the complete terminal path:
#PS1='${debian_chroot:+($debian_chroot)}\[\033[01;${bash_color}m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00;32m\] $(git_branch)\[\033[00m\]\$ '

#select this one to show the last folder in the path:
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;${bash_color}m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\W\[\033[00;32m\] $(git_branch)\[\033[00m\]\$ '
