from dataclasses import dataclass
import subprocess
import telnetlib
import secrets
import dotenv
import shutil
import time
import sys
import os

@dataclass
class RTKRCV:
    """Wrapper class for ``rtklib``'s ``rtkrcv`` command. Designed to be used
    within a ``with`` block which creates and destroys the ``rtkrcv`` thread
    nicely. e.g.:

    .. code-block:: python

        with RTKRCV(sys.argv[1]) as rtkrcv:
            print(rtkrcv.status())
            time.sleep(2)

    This wrapper class communicates with the ``rtkrcv`` thread with the telnet protocol.
    The ``rtkrcv`` is started at port ``telnet_port`` with password ``telnet_passwd`` which
    is 2101 and randomly generated respectively, but this can be overridden. ``autostart``
    automatically runs the `start` command after instansiation. 

    Arguments:
        conf_path (str): Path to the ``rtkrcv`` config path to use. Also see :func:`start`
        telnet_port (int): Path for the telnet port for communication (default: 2120)
        telnet_passwd (str): Password for telnet communication. Generated randomly by default
        autostart (bool): If set to ``True``, :func:`start` will be run automatically after instansiation

    .. todo::

        At the moment, arguments for ``rtkrcv`` commands are not yet implemented, only the command alone

    .. warning::

        This class uses :external+python:py:class:`telnetlib.Telnet`, which will be removed in Python 3.13.
    """

    conf_path: str
    telnet_port: int = 2120
    telnet_passwd: str = secrets.token_hex(8)
    autostart: bool = True

    def __enter__(self):

        # if the log files in the RTKRCV config file already exist on disk,
        # in the telnet console when the `start` command is run, it will interactively
        # ask if the user wants to overwrite these files. instead of handling this,
        # just delete the log files before we start
        self.config = dotenv.dotenv_values(self.conf_path)
        for k, v in self.config.items():
            if "logstr" in k and "path" in k:
                if os.path.exists(v.replace("::T", "")):
                    os.remove(v.replace("::T", ""))
                    print("Removed '%s'" % v.replace("::T", ""))

        cmd = [shutil.which("rtkrcv"), "-o", self.conf_path, "-p", str(self.telnet_port), "-w", self.telnet_passwd]
        print("Attempting RTKRTV startup with command '%s'" % " ".join(cmd))

        self.proc = subprocess.Popen(
            cmd,
            stdout = subprocess.PIPE,
            stdin = subprocess.PIPE,
            stderr = subprocess.PIPE
        )
        time.sleep(1)

        if self.autostart:
            self.start()
        
        return self

    def __exit__(self, type, value, traceback):
        self._send_and_respond("shutdown")

    def _send_and_respond(self, message: str):
        with telnetlib.Telnet(host = "localhost", port = self.telnet_port) as tn:
            tn.read_until(b"password: ")
            tn.write(self.telnet_passwd.encode("ascii") + b"\r\n")

            tn.read_until(b"rtkrcv> ")
            tn.write(message.encode("ascii") + b"\r\n")
            if message == "shutdown":
                print("Shutdown command sent")
                time.sleep(0.5)
                self.proc.kill()
                return
            resp = tn.read_until(b"rtkrcv> ")

            return resp.decode()

    def help(self):
        return self._send_and_respond("help")

    def start(self):
        """
        start                 : start rtk server
        

        .. warning::

            So that this can be executed without interaction, config option keys containing the
            strings ``"logstr"`` and ``"path"`` that have a value which is a file on disk will
            be removed.
        """
        return self._send_and_respond("start")

    def stop(self):
        """stop                  : stop rtk server"""
        return self._send_and_respond("stop")

    def restart(self):
        """restart               : restart rtk sever"""
        return self._send_and_respond("restart")

    def solution(self):
        """solution [cycle]      : show solution"""
        return self._send_and_respond("solution")

    def status(self):
        """status [cycle]        : show rtk status"""
        return self._send_and_respond("status")

    def satellite(self):
        """satellite [-n] [cycle]: show satellite status"""
        return self._send_and_respond("satellite")

    def observ(self):
        """observ [-n] [cycle]   : show observation data"""
        return self._send_and_respond("observ")

    def navidata(self):
        """navidata [cycle]      : show navigation data"""
        return self._send_and_respond("navidata")

    def stream(self):
        """stream [cycle]        : show stream status"""
        return self._send_and_respond("stream")

    def ssr(self):
        """ssr [cycle]           : show ssr corrections"""
        return self._send_and_respond("ssr")

    def error(self):
        """error                 : show error/warning messages"""
        return self._send_and_respond("error")

    def option(self):
        """option [opt]          : show option(s)"""
        return self._send_and_respond("option")

    def set_opt(self):
        """set opt [val]         : set option"""
        return self._send_and_respond("set opt")

    def load(self):
        """load [file]           : load options from file"""
        return self._send_and_respond("load")

    def save(self):
        """save [file]           : save options to file"""
        return self._send_and_respond("save")

    def log(self):
        """log [file|off]        : start/stop log to file"""
        return self._send_and_respond("log")

if __name__ == "__main__":
    with RTKRCV(sys.argv[1]) as rtkrcv:
        print(rtkrcv.help())
        time.sleep(20)
        print(rtkrcv.status())
        time.sleep(2000)

