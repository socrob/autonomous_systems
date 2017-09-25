function roscat {
    local arg
    if [[ $1 = "--help" ]] || [[ $# -ne 2 ]]; then
       echo -e "usage: roscat [package] [file]\n\nDisplay a file content within a package."
       [[ $1 = "--help" ]] && return 0 || return 1
    fi
    _roscmd ${1} ${2}
    [ $? -eq 1 ] && return 1
    if [[ -n ${arg} ]]; then
        if [[ -z $CATTER ]]; then
            cat ${arg}
        else
            $CATTER ${arg}
        fi
    fi
}
