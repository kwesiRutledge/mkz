# $1 is the session name you want to replay.
if [ $# -eq 0 ]
  then
    echo "No arguments supplied"
elif [ $# -eq 1 ]
  then
    ./polysync-rnr-control-c -t $1
elif [ $# -eq 2 ]
  then  # $2 is the relative start time (microseconds)
    ./polysync-rnr-control-c -t $1 -s $2
fi

