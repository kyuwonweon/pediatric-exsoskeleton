
import micropython                                  # type: ignore
import pyb                                          # type: ignore
from lib import config
from lib.ll_common import mode_switch
from lib.IO.ReporterInterface import Reporter
from lib.CtrlFactory.CtrlFactory import CtrlFactory

class ReporterStdOut(Reporter):
    def __init__(self, ctrlFact:CtrlFactory, params_file:str):
        super().__init__(ctrlFact, params_file)

        self.start_timer()


    def reporter(self,_):
        print(' | '.join(self.create_report()))
