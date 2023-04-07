source /opt/ros/noetic/setup.bash
function rosdep_install_all() {
    set -o xtrace
    sudo apt-get update && rosdep update && \
        rosdep install --from-paths src --ignore-src -r -y
    { set +o xtrace; } 2>/dev/null
}
function source_devel() {
    if [ -f devel/setup.bash ]; then
        source devel/setup.bash;
    else
        echo "No devel/setup.bash found. This must be the first time you are building the workspace. Please run 'catkin build'.";
    fi
}
function devsetup() { cd /workspace/catkin_ws && source_devel; }

function rrecord() {
    __name_or_path="${1:-unnamed}"
    shift

    # if the argument ends with .bag, then it is a full path
    if [[ "$__name_or_path" == *.bag ]]; then
        __output_bag_path="$__name_or_path"
    else
        __output_bag_path="./$(date --iso-8601=seconds)-$__name_or_path.bag"
    fi

    echo "Recording to \"$__output_bag_path\""

    rosbag record -j -a -O "$__output_bag_path" $@
}

function slugify() {
    echo "$1" | iconv -t ascii//TRANSLIT | sed -r s/[~\^]+//g | sed -r s/[^a-zA-Z0-9]+/-/g | sed -r s/^-+\|-+$//g | tr A-Z a-z
}

alias ds='devsetup'

export ROSCONSOLE_FORMAT='[${severity}] - ${node}: [${time}] ${message}'

# Set up the ROS environment
devsetup