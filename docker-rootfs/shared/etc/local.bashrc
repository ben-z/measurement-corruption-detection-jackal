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

alias ds='devsetup'
