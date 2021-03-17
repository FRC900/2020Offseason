#!/usr/bin/env python


#############################################################################
##
## Copyright (C) 2013 Riverbank Computing Limited.
## Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
## All rights reserved.
##
## This file is part of the examples of PyQt.
##
## $QT_BEGIN_LICENSE:BSD$
## You may use this file under the terms of the BSD license as follows:
##
## "Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are
## met:
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above copyright
##     notice, this list of conditions and the following disclaimer in
##     the documentation and/or other materials provided with the
##     distribution.
##   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
##     the names of its contributors may be used to endorse or promote
##     products derived from this software without specific prior written
##     permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
## A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
## OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
## SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
## LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
## DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
## THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
## $QT_END_LICENSE$
##
#############################################################################


from PyQt5.QtCore import QDir, QPoint, QRect, QSize, Qt
from PyQt5.QtGui import QImage, QImageWriter, QPainter, QPen, qRgb
from PyQt5.QtWidgets import (QAction, QApplication, QColorDialog, QFileDialog,
        QInputDialog, QMainWindow, QMenu, QMessageBox, QWidget)
from PyQt5.QtPrintSupport import QPrintDialog, QPrinter


class ScribbleArea(QWidget):
    def __init__(self, parent=None):
        super(ScribbleArea, self).__init__(parent)

        self.setAttribute(Qt.WA_StaticContents)
        self.modified = False
        self.scribbling = False
        self.myPenWidth = 5
        self.myPenColor = Qt.blue
        self.image = QImage()
        self.current_image = None
        self.lastPoint = QPoint()
        self.enable_drawing = True
        self.coords = list()

    def openImage(self, fileName):

        if fileName is None:
            return False

        self.coords = list()
        loadedImage = QImage()
        if not loadedImage.load(fileName):
            return False

        newSize = loadedImage.size().expandedTo(self.size())
        self.resizeImage(loadedImage, newSize)
        self.image = loadedImage
        self.modified = False

        # Set new fixed size based on the loaded image.
        self.setFixedWidth(loadedImage.width())
        self.setFixedHeight(loadedImage.height())
        self.current_image = fileName

        self.update()
        self.myPenColor = Qt.blue
        return True

    def enableDrawing(self, enable):
        self.enable_drawing = enable
        if not enable:
            self.reloadImage()

    def reloadImage(self):
        self.clearImage()
        self.openImage(self.current_image)
    
    def saveImage(self, fileName, fileFormat):
        visibleImage = self.image
        self.resizeImage(visibleImage, self.size())

        if visibleImage.save(fileName, fileFormat):
            self.modified = False
            return True
        else:
            return False

    def setPenColor(self, newColor):
        self.myPenColor = newColor

    def setPenWidth(self, newWidth):
        self.myPenWidth = newWidth

    def unsetImage(self):
        self.current_image = None

    def clearImage(self):
        self.coords = list()
        self.image.fill(qRgb(255, 255, 255))
        self.modified = True
        # self.current_image = None
        self.update()

    def GetCoords(self):
        return self.coords

    def mousePressEvent(self, event):

        if not self.enable_drawing:
            return

        if event.button() == Qt.LeftButton:
            self.scribbling = True
            drawSuccess = self.drawPoint(event.pos())
            if not drawSuccess:
                return

            if self.coords:
                self.drawLineTo(event.pos())

            coordinate = event.pos()
            self.coords.append((coordinate.x(), coordinate.y()))
            self.lastPoint = event.pos()


    def paintEvent(self, event):
        painter = QPainter(self)
        dirtyRect = event.rect()
        painter.drawImage(dirtyRect, self.image, dirtyRect)

    def resizeEvent(self, event):
        if self.width() > self.image.width() or self.height() > self.image.height():
            newWidth = max(self.width() + 128, self.image.width())
            newHeight = max(self.height() + 128, self.image.height())
            self.resizeImage(self.image, QSize(newWidth, newHeight))
            self.update()

        super(ScribbleArea, self).resizeEvent(event)

    def drawPoint(self, point):
        painter = QPainter(self.image)

        # Don't paint anything if the image is showing black.
        img_rgb = self.image.pixel(point)
        # print('{0:X} -> {1:d}'.format(img_rgb, img_rgb & 0x00FFFFFF))
        if img_rgb & 0x00FFFFFF == 0:
            return False

        painter.setPen(QPen(Qt.blue, self.myPenWidth, Qt.SolidLine,
                Qt.RoundCap, Qt.RoundJoin))
        
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.drawPoint(point)


        self.modified = True

        rad = self.myPenWidth / 2 + 2
        self.update()
        # self.update(QRect(self.lastPoint, point).normalized().adjusted(-rad, -rad, +rad, +rad))
        # self.lastPoint = QPoint(point)
        return True

    def drawLineTo(self, endPoint):
        painter = QPainter(self.image)
        painter.setPen(QPen(Qt.blue, self.myPenWidth, Qt.SolidLine,
                Qt.RoundCap, Qt.RoundJoin))
        painter.drawLine(self.lastPoint, endPoint)
        self.modified = True

        rad = self.myPenWidth / 2 + 2
        self.update()
        # self.update(QRect(self.lastPoint, endPoint).normalized().adjusted(-rad, -rad, +rad, +rad))
        self.lastPoint = QPoint(endPoint)

    def resizeImage(self, image, newSize):
        if image.size() == newSize:
            return

        newImage = QImage(newSize, QImage.Format_RGB32)
        newImage.fill(qRgb(255, 255, 255))
        painter = QPainter(newImage)
        painter.drawImage(QPoint(0, 0), image)
        self.image = newImage

    def isModified(self):
        return self.modified
