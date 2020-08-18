from iocbuilder import Device, AutoSubstitution, Architecture, SetSimulation
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP

from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, NDFileTemplate, makeTemplateInstance, includesTemplates

@includesTemplates(ADBaseTemplate, NDFileTemplate)
class tucsenTemplate(AutoSubstitution):
#    TemplateFile = "andorCCD.template"
#    SubstitutionOverwrites = [_ADBaseTemplate]
    TemplateFile = "tucsen.template"
#    SubstitutionOverwrites = [_NDFile]

class tucsenSpecificMustBeLoadedFirst(Device):
    """Library dependencies for areaDetector"""
    # Install different Andor libraries, depending on the target platform.
    LibFileList = ['tucsen']
    AutoInstantiate = True

class tucsen(AsynPort):
    """Creates a Tucsen camera areaDetector driver"""
    Dependencies = (tucsenSpecificMustBeLoadedFirst, ADCore)
    # This tells xmlbuilder to use PORT instead of name as the row ID
    UniqueName = "PORT"
    _SpecificTemplate = tucsenTemplate
    def __init__(self, PORT, TRACEMASK, CAMERAID=0, BUFFERS = 50, MEMORY = 0, PRIORITY = 0, STACKSIZE = 100000, **args):
        # Init the superclass (AsynPort)
        self.__super.__init__(PORT)
        # Update the attributes of self from the commandline args
        self.__dict__.update(locals())
        # Make an instance of our template
        makeTemplateInstance(self._SpecificTemplate, locals(), args)
        locals().update(args)

    # __init__ arguments
    ArgInfo = ADBaseTemplate.ArgInfo + _SpecificTemplate.ArgInfo + makeArgInfo(__init__,
        PORT = Simple('Port name for the detector', str),
        CAMERAID = Simple('cameraID The index number of the Shamrock spectrograph, if installed.', int),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int),
        PRIORITY = Simple('The thread priority for the asyn port driver thread', int),
        STACKSIZE = Simple('The stack size for the asyn port driver thread', int),
        TRACEMASK = Simple('TraceMask', int))

    # Device attributes
    DbdFileList = ['tucsenSupport']

    # Install different Andor libraries, depending on the target platform.
    LibFileList = ['tucsen', 'TUCam']

    def Initialise(self):
        print '# tucsenConfig(portName, cameraId, traceMask, maxBuffers, maxMemory, priority, stackSize)'
        print 'tucsenConfig("%(PORT)s", "%(CAMERAID)s", %(TRACEMASK)d, %(BUFFERS)d, %(MEMORY)d, %(PRIORITY)d, %(STACKSIZE)d)' \
            % self.__dict__

