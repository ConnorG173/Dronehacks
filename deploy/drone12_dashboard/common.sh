# Shared settings for the drone-12 dashboard kit. Host paths; the container sees ~/team_ws as /root/team_ws.
KIT_HOST="$HOME/team_ws/team_staging/conservative_v1"
KIT_CT="/root/team_ws/team_staging/conservative_v1"
CT=skybeta-stack
TMUX_S=skybeta
ENV_CT="source skybeta_env.sh >/dev/null 2>&1; source $KIT_CT/install/setup.bash"
ct() { docker exec "$CT" bash -lc "$1"; }
