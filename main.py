import sys
from PySide6.QtWidgets import QApplication
from Windows.opta_window import OptaWindow

def main():
    app = QApplication(sys.argv)
    window = OptaWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
