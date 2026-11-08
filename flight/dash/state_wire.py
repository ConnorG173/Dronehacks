"""Compact HTTP snapshots without weakening telemetry ages or control checks."""


def compact_state(state, versions=None):
    """Mutate a fresh snapshot; omit only geometry the client already has.

    Version keys include process identity and invalidation state. Never suppress
    live telemetry, metadata, age or stale flags. Course-coordinate copies are
    redundant for this browser: it draws the world arrays and has explicit counts.
    """
    versions = versions or {}
    depth = state.get('depth')
    if isinstance(depth, dict):
        same = bool(versions.get('dg') and versions['dg'] == depth.get('geometry_version'))
        depth['geometry_unchanged'] = same
        if same:
            for key in ('points', 'camera_xy', 'view_polygon'):
                depth.pop(key, None)
    course = state.get('course')
    if isinstance(course, dict):
        course.pop('original_course', None)
        course.pop('adjusted_course', None)
        for prefix, query in (('original','co'),('adjusted','ca')):
            same = bool(versions.get(query) and versions[query] == course.get(prefix+'_version'))
            course[prefix+'_unchanged'] = same
            if same:
                course.pop(prefix+'_world_enu', None)
                course.pop(prefix+'_yaw_enu_rad', None)
    return state
