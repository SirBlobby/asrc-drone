import getpass
import os
import socket
import time

import config


class CsvWriter:
    def __init__(self, path, fields):
        self.fields = fields
        self.handle = open(path, "w", buffering=1)
        self.handle.write(",".join(fields) + "\n")

    def write(self, **row):
        values = [str(row.get(field, "")) for field in self.fields]
        self.handle.write(",".join(values) + "\n")

    def flush(self):
        self.handle.flush()

    def download_command(self):
        return (f"scp -r {_remote_host()}:{os.path.abspath(self.directory)} "
                f"{config.SCP_DESTINATION}")

    def close(self):
        self.handle.close()


class FlightLog:
    def __init__(self, root="logs", tag="clump"):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.directory = os.path.join(root, f"{tag}_{stamp}")
        os.makedirs(self.directory, exist_ok=True)
        self.events = open(os.path.join(self.directory, "events.log"),
                           "w", buffering=1)
        self.writers = []
        self.started = time.monotonic()

    def csv(self, name, fields):
        writer = CsvWriter(os.path.join(self.directory, f"{name}.csv"), fields)
        self.writers.append(writer)
        return writer

    def event(self, source, message):
        line = f"{time.monotonic() - self.started:8.2f} [{source}] {message}"
        print(line, flush=True)
        self.events.write(line + "\n")

    def download_command(self):
        return (f"scp -r {_remote_host()}:{os.path.abspath(self.directory)} "
                f"{config.SCP_DESTINATION}")

    def close(self):
        for writer in self.writers:
            writer.close()
        self.events.close()


def _remote_host():
    if config.SCP_HOST:
        return config.SCP_HOST
    name = socket.gethostname()
    suffix = "" if "." in name else ".local"
    return f"{getpass.getuser()}@{name}{suffix}"
