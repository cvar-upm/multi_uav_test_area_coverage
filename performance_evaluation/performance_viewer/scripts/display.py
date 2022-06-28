#!/usr/bin/env python

import sys
from performance_viewer.MainWindow import MainWindow
from PySide6.QtWidgets import QApplication

if __name__ == "__main__":
    app = QApplication([])
    app.setApplicationName("Performance viewer")

    widget = MainWindow()
    widget.show()

    sys.exit(app.exec())