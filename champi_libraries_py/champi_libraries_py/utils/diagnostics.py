import diagnostic_msgs
import diagnostic_updater

import time


class ExecTimeMeasurer:
    """Class to measure an execution time of a code block located between start() and stop() calls."""
    def __init__(self):
        self.start_time = None
        self.exec_time = None
        self.worst_exec_time = None


    def start(self):
        """Call this method before the code block you want to measure.
        """
        self.start_time = time.time()


    def stop(self):
        """Call this method after the code block you want to measure.
        """
        self.exec_time = time.time() - self.start_time
        if self.worst_exec_time is None or self.exec_time > self.worst_exec_time:
            self.worst_exec_time = self.exec_time


    def produce_diagnostics(self, stat):
        """Callback method to pass to a diagnostic updater. It will produce diagnostics about the measured execution time.

        Args:
            stat (_type_): No need to worry about that, the diagnostic updater takes care of it.

        Returns:
            _type_:  No need to worry about that, the diagnostic updater takes care of it.
        """
        if self.exec_time is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Not measured yet')
            stat.add('Exec time (ms)', 'Not measured yet')
            stat.add('Worst exec time (ms)', 'Not measured yet')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Exec time measured')
            stat.add('Exec time (ms)', str(self.exec_time*1000))
            stat.add('Worst exec time (ms)', str(self.worst_exec_time*1000))
        return stat


def create_topic_freq_diagnostic(name, updater, expected_freq, tolerance=0.1):
    """Wrapper function that creates and registers a task, that that checks the frequency of a topic

    Args:
        name (string): Name of the topic (for the title of the diagnostic message).
        updater (DiagnosticUpdater): Diagnostic updater to which the task will be added.
        expected_freq (float): Expected frequency of the topic (Hz).
        tolerance (float, optional): Tolerance to the expected frequency to consider the frequency is within bound.
        It's expressed as a ratio of the expected frequency. Defaults to 0.1.

    Returns:
        HeaderlessTopicDiagnostic: An object that takes care of the frequency check. You need to call its tick() method
        when you publish a message on the topic, to update the statistics.
    """
    freq_range = {'min': expected_freq, 'max': expected_freq}
    return diagnostic_updater.HeaderlessTopicDiagnostic(name, updater, diagnostic_updater.FrequencyStatusParam(freq_range, tolerance, 10))  