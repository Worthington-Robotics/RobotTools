from diagnostic_msgs.msg import DiagnosticStatus
import diagnostic_updater
import psutil
import rclpy

class CpuTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "CPU Information")
        self._warning_percentage = int(warning_percentage)


    def run(self, stat):
        cpu_percentages = (psutil.cpu_percent(percpu=True))
        cpu_average = sum(cpu_percentages) / len(cpu_percentages)

        stat.add("CPU Load Average", "{:.2f}".format(cpu_average))

        for idx, val in enumerate(cpu_percentages):
            stat.add("CPU {} Load".format(idx), "{:.2f}".format(val))

        if cpu_average > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Average CPU usage exceeds {:d}%".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Average CPU utilization {:.2f}%".format(cpu_average))

        return stat

class MemoryTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Memory Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        memory_info = psutil.virtual_memory()

        stat.add("Total Memory", "{:.2f} GB".format(memory_info.total / (1024.0 ** 3)))
        stat.add("Used Memory", "{:.2f} GB".format(memory_info.used / (1024.0 ** 3)))
        stat.add("Percent Used", "{:.2f}%".format(memory_info.percent))

        if memory_info.percent > self._warning_percentage:
            stat.summary(DiagnosticStatus.WARN,
                         "Memory usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Memory usage {:.2f}%".format(memory_info.percent))

        return stat

class DiskTask(diagnostic_updater.DiagnosticTask):
    def __init__(self, warning_percentage):
        diagnostic_updater.DiagnosticTask.__init__(self, "Disk Information")
        self._warning_percentage = int(warning_percentage)

    def run(self, stat):
        all_partitions = psutil.disk_partitions()

        warn = False
        for partition in all_partitions:
            if "snap" in partition.mountpoint:
                continue
            if "loop" in partition.device:
                continue

            disk_usage = psutil.disk_usage(partition.mountpoint)

            used = disk_usage.used / (1024.0 ** 3)
            total = disk_usage.total / (1024.0 ** 3)
            stat.add("\"{:s}\" Usage".format(partition.mountpoint), "{:.2f} GB / {:.2f} GB ({:.2f}%)".format(used, total, disk_usage.percent))

            if disk_usage.percent > self._warning_percentage:
                warn = True


        if warn:
            stat.summary(DiagnosticStatus.WARN,
                         "Disk usage exceeds {:d} percent".format(self._warning_percentage))
        else:
            stat.summary(DiagnosticStatus.OK, "Disk usage is OK")

        return stat

def main():
    rclpy.init()
    node = rclpy.create_node('computer_monitor')
    print('Hi from diagnostics.')

    updater = diagnostic_updater.Updater(node)
    updater.add(CpuTask(90))
    updater.add(MemoryTask(90))
    updater.add(DiskTask(90))
    updater.force_update()
    rclpy.spin(node, None)


if __name__ == '__main__':
    main()
