import sys
from PyQt5.QtWidgets import QApplication
from Windows.selection_window import SelectionWindow

def main():
    app = QApplication(sys.argv)
    window = SelectionWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
