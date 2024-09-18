

def determine_actions(route):
    actions = []
    for i in range(len(route) - 1):
        current_wp = route[i]
        next_wp = route[i + 1]

        # Calculate yaw difference
        yaw_current = current_wp.transform.rotation.yaw % 360
        yaw_next = next_wp.transform.rotation.yaw % 360
        yaw_diff = (yaw_next - yaw_current + 180) % 360 - 180  # Normalize to [-180, 180]

        if abs(yaw_diff) > 10:
            action = 'Left' if yaw_diff > 0 else 'Right'
        else:
            action = 'Forward'

        actions.append(action)

        # Append 'Stop' at the end
    actions.append('Stop')
    return actions