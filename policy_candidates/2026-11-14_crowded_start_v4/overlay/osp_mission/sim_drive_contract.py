"""Check the simulator's actual actuator path, excluding evaluator subscribers."""


def errors(observed):
    result = []
    if 'motion_control' not in observed.get('velocity_consumer_nodes', []):
        result.append('drive path: motion_control does not consume cmd_vel')
    if observed.get('drive_publisher_nodes', []).count('motion_control') != 1:
        result.append('drive path: require one motion_control output publisher')
    if 'diffdrive_controller' not in observed.get('drive_consumer_nodes', []):
        result.append('drive path: actuator subscriber unavailable')
    heartbeat = observed.get('drive_heartbeat', {})
    if heartbeat.get('count', 0) < 3 or not 0 <= heartbeat.get('age_s', 999) <= 1.:
        result.append('drive path: no recent actuator command stream')
    return result
