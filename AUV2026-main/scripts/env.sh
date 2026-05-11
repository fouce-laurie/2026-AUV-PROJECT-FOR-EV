#!/usr/bin/env bash
# Unified UUV workspace environment helper.
# Usage:
#   source scripts/env.sh
# Then use:
#   uuv_env_print
#   uuv_sim [extra ros2 launch args]
#   uuv_pool [extra ros2 launch args]
#   uuv_real [extra ros2 launch args]

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Please source this file instead of executing it:"
  echo "  source scripts/env.sh"
  exit 1
fi

_uuv_env_warn() {
  echo "[uuv-env] $*" >&2
}

_uuv_env_source_if_exists() {
  local file_path="$1"
  if [[ -f "$file_path" ]]; then
    # Avoid leaking caller positional args into sourced scripts.
    set --
    # shellcheck disable=SC1090
    source "$file_path"
    return 0
  fi
  return 1
}

uuv_source() {
  local script_dir repo_root workspace_root
  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  repo_root="$(cd -- "$script_dir/.." && pwd)"
  workspace_root="$repo_root"

  export UUV_REPO_ROOT="$repo_root"
  export UUV_WORKSPACE_ROOT="$workspace_root"
  export UUV_DATAS_DIR="$workspace_root/datas"
  export UUV_STONEFISH_DATA_DIR="$workspace_root/src/stonefish_ros2/Data"
  export UUV_SCENARIO_DEFAULT="$workspace_root/src/stonefish_ros2/Data/underwater_xunyun.scn"
  export UUV_SCENARIO_QUALIFICATION="$workspace_root/src/stonefish_ros2/Data/sauvc_2026_qualification.scn"
  export UUV_SCENARIO_FINALS="$workspace_root/src/stonefish_ros2/Data/sauvc_2026_finals.scn"
  export UUV_SCENARIO_TEST="$UUV_SCENARIO_DEFAULT"

  # Avoid Fast DDS shared-memory lock conflicts (fastrtps_port* in /dev/shm).
  # Keep overridable by user if a different transport setup is needed.
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

  # Optional conda activation (set UUV_SKIP_CONDA=1 to disable).
  if [[ "${UUV_SKIP_CONDA:-0}" != "1" ]]; then
    if [[ -f "$HOME/miniconda3/bin/activate" ]]; then
      # shellcheck disable=SC1090
      source "$HOME/miniconda3/bin/activate" ros2_jazzy_env >/dev/null 2>&1 || _uuv_env_warn "conda env ros2_jazzy_env not found; continue without it"
    else
      _uuv_env_warn "miniconda activate script not found at $HOME/miniconda3/bin/activate"
    fi
  fi

  _uuv_env_source_if_exists "/opt/ros/jazzy/setup.bash" || _uuv_env_warn "ROS Jazzy setup not found: /opt/ros/jazzy/setup.bash"

  if [[ -f "$workspace_root/install/local_setup.bash" ]]; then
    # Source local_setup to avoid chaining unrelated underlays from generated setup.bash.
    # shellcheck disable=SC1090
    source "$workspace_root/install/local_setup.bash"
  elif [[ -f "$workspace_root/install/setup.bash" ]]; then
    # Fallback for older build layouts.
    # shellcheck disable=SC1090
    source "$workspace_root/install/setup.bash"
  else
    _uuv_env_warn "install/{local_setup.bash,setup.bash} not found; run colcon build in $workspace_root"
  fi

  # Optional venv site-packages compatibility for ros2 launch entrypoints.
  if [[ -d "$workspace_root/.venv/lib" ]]; then
    local sp
    sp="$(find "$workspace_root/.venv/lib" -maxdepth 2 -type d -name site-packages 2>/dev/null | head -n 1)"
    if [[ -n "$sp" ]]; then
      case ":${PYTHONPATH:-}:" in
        *":$sp:"*) ;;
        *) export PYTHONPATH="$sp${PYTHONPATH:+:$PYTHONPATH}" ;;
      esac
    fi
  fi

  return 0
}

uuv_env_print() {
  echo "UUV_REPO_ROOT=$UUV_REPO_ROOT"
  echo "UUV_WORKSPACE_ROOT=$UUV_WORKSPACE_ROOT"
  echo "UUV_DATAS_DIR=$UUV_DATAS_DIR"
  echo "UUV_STONEFISH_DATA_DIR=$UUV_STONEFISH_DATA_DIR"
  echo "UUV_SCENARIO_DEFAULT=$UUV_SCENARIO_DEFAULT"
  echo "UUV_SCENARIO_QUALIFICATION=$UUV_SCENARIO_QUALIFICATION"
  echo "UUV_SCENARIO_FINALS=$UUV_SCENARIO_FINALS"
}

uuv_sim() {
  local scenario_type="${1:-test}"
  local scenario_file
  shift || true  # Remove first arg from $@

  # Parse optional helper flags (e.g. --enable-depth) and forward remaining args to ros2 launch
  local enable_depth_flag=0
  local -a pass_args=()
  for _arg in "$@"; do
    if [[ "$_arg" == "--enable-depth" ]]; then
      enable_depth_flag=1
    else
      pass_args+=("$_arg")
    fi
  done
  
  case "$scenario_type" in
    qualification)
      scenario_file="$UUV_SCENARIO_QUALIFICATION"
      ;;
    finals)
      scenario_file="$UUV_SCENARIO_FINALS"
      ;;
    test)
      scenario_file="$UUV_SCENARIO_TEST"
      ;;
    *)
      _uuv_env_warn "Unknown scenario type: $scenario_type. Use 'qualification', 'finals', or 'test'. Defaulting to 'test'."
      scenario_file="$UUV_SCENARIO_TEST"
      ;;
  esac
  
  # If --enable-depth was provided, translate to a launch argument
  if [[ "$enable_depth_flag" -eq 1 ]]; then
    pass_args+=("enable_depth:=true")
  fi

  # Default window size adjustment (can still be overridden by passing window_res_x:=... in CLI)
  local -a window_args=()
  [[ " ${pass_args[*]} " != *" window_res_x:="* ]] && window_args+=("window_res_x:=1920")
  [[ " ${pass_args[*]} " != *" window_res_y:="* ]] && window_args+=("window_res_y:=1080")

  ros2 launch uv_launch_pkg sim_launch.py \
    simulation_data:="$UUV_STONEFISH_DATA_DIR" \
    scenario_desc:="$scenario_file" \
    "${window_args[@]}" \
    "${pass_args[@]}"
}

uuv_real() {
  ros2 launch uv_launch_pkg real_test.launch.py "$@"
}

uuv_help() {
  cat <<'EOF'
UUV Environment Helpers

1. source scripts/env.sh
2. uuv_env_print
3. uuv_sim <scenario> [launch args...]
4. uuv_real [args]

Scenario types for uuv_sim:
  qualification - SAUVC 2026 qualification arena
  finals        - SAUVC 2026 finals arena
  test          - Default underwater test scenario (default if not specified)

Examples:
  uuv_sim qualification
  uuv_sim finals
  uuv_sim test
  uuv_sim finals enable_ai:=true
  uuv_real

Optional:
  export UUV_SKIP_CONDA=1
  source scripts/env.sh
EOF
}

# Initialize immediately on source.
uuv_source
_uuv_env_warn "Environment loaded. Use uuv_help for shortcuts."
