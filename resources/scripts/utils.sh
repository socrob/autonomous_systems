function get_interface_that_pings()
{
	local ip

	# accept piping
	if (( $# == 0 )) ;then
		ip=$(</dev/stdin)
	else
		ip=$1
	fi

    local interface=`ip -o route get $ip | perl -nle 'if ( /dev\s+(\S+)/ ) {print $1}'`
	echo $interface
}

function get_ip_of_interface()
{
	local interface

	# accept piping
	if (( $# == 0 )) ; then
		interface=$(</dev/stdin)
	else
		interface=$1
	fi

	local ip=`ip -f inet addr show $interface | grep -Po 'inet \K[\d.]+'`
	echo $ip
}
