"""Fake adsk.core: application, UI dialogs and enums the exporter uses."""


class DialogResults:
    DialogOK = 0
    DialogYes = 2
    DialogNo = 3


class MessageBoxButtonTypes:
    YesNoButtonType = 3


class MessageBoxIconTypes:
    WarningIconType = 2


class _ProgressDialog:
    isCancelButtonShown = True
    message = ""
    progressValue = 0

    def show(self, *args):
        pass

    def hide(self):
        pass


class _UserInterface:
    """Scriptable answers: set folder / robot_name / overwrite before a run."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.folder = ""
        self.robot_name = "robot"
        self.overwrite = True
        self.cancel_folder = False
        self.cancel_name = False
        self.messages = []

    def createFolderDialog(self):
        ui = self

        class _FolderDialog:
            title = ""
            folder = ui.folder

            def showDialog(self):
                return 1 if ui.cancel_folder else DialogResults.DialogOK

        return _FolderDialog()

    def inputBox(self, prompt, title, default):
        return (self.robot_name, self.cancel_name)

    def messageBox(self, text, title="", *args):
        self.messages.append(text)
        return DialogResults.DialogYes if self.overwrite else DialogResults.DialogNo

    def createProgressDialog(self):
        return _ProgressDialog()


class _Document:
    name = "Robot v1"


class _Application:
    def __init__(self):
        self.userInterface = _UserInterface()
        self.activeProduct = None
        self.activeDocument = _Document()
        self.logs = []

    def log(self, text):
        self.logs.append(text)


_app = _Application()


class Application:
    @staticmethod
    def get():
        return _app
