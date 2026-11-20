"""use_course and set_model name every course prop no running model emits."""
from mongla_localization.course_map import Course, Prop
from mongla_planner.dsl_heading import HeadingMixin


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warning(self, m):
        self.lines.append(('warning', m))


class _M(HeadingMixin):
    def __init__(self, classes):
        self.log = _Log()
        self._classes = classes
        self._course = Course('robosub26', {
            'gate': Prop('gate', detect_class='gate'),
            'pipe': Prop('pipe', detect_class='red_pipe'),
            'bin': Prop('bin')})

    def model_classes(self, camera, *, timeout=0.5):
        return self._classes.get(camera)


def test_props_no_model_emits_are_warned_by_name():
    m = _M({'forward': {'gate', 'shark'}, 'downward': {'bin'}})
    assert m._check_course_classes() == ['pipe']
    warn = [t for k, t in m.log.lines if k == 'warning']
    assert warn and 'pipe (red_pipe)' in warn[0]


def test_every_class_emitted_is_silent():
    m = _M({'forward': {'gate', 'red_pipe'}, 'downward': {'bin'}})
    assert m._check_course_classes() == []
    assert not [t for k, t in m.log.lines if k == 'warning']


def test_no_detector_running_says_it_could_not_check():
    m = _M({})
    assert m._check_course_classes() == []
    assert any('not checked' in t for _, t in m.log.lines)
