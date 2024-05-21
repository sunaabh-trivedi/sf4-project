from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QMainWindow
from PyQt6.QtGui import QPixmap
import sys

class WeatherUI(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Weather Balloon Data")
        self.setGeometry(500, 500, 1200, 600)
        self.setStyleSheet("background-color:#4f5250")

        layout = QHBoxLayout()

        mainWidget = QWidget()
        mainWidget.setLayout(layout)
        self.setCentralWidget(mainWidget)


class AltitudeWidget(QWidget):

    def __init__(self):
        pass

class WindWidget(QWidget):
    
    def __init__(self):
        pass

class TemperatureWidget(QWidget):

    def __init__(self):
        pass

class HumidityWidget(QWidget):

    def __init__(self):
        label = QLabel(self)
        pixmap = QPixmap("humidity.png")
        label.setPixmap(pixmap)
        self.resize(pixmap.width(), pixmap.height())

class TempHumidity(QWidget):

    def __init__(self):

        layout = QVBoxLayout()
        layout.addWidget(TemperatureWidget())
        layout.addWidget(HumidityWidget())

        self.addLayout(layout)


def main():

    app = QApplication([])
    weather_ui = WeatherUI()
    weather_ui.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()