#!/usr/bin/python

#
# @file
#
# @section LICENSE
#
# Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
#
# This file is part of SCRIMMAGE.
#
#   SCRIMMAGE is free software: you can redistribute it and/or modify it under
#   the terms of the GNU Lesser General Public License as published by the
#   Free Software Foundation, either version 3 of the License, or (at your
#   option) any later version.
#
#   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
#   License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
#
# @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
# @author Eric Squires <eric.squires@gtri.gatech.edu>
# @date 31 July 2017
# @version 0.1.0
# @brief Brief file description.
# @section DESCRIPTION
# A Long description goes here.
#
#

import matplotlib as mpl
mpl.use("Agg")
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wx import NavigationToolbar2Wx
from matplotlib.figure import Figure

import wx
import wx.lib.agw.customtreectrl as CT
import pandas as pd
import os
import scrimmage as sc


class CanvasFrame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, -1, 'Scrimmage Plotting Tool')
        self.figure = Figure()
        self.ax = self.figure.add_subplot(111)

        self.canvas = FigureCanvas(self, -1, self.figure)

        self.toolbar = NavigationToolbar2Wx(self.canvas)
        self.toolbar.Realize()

        self.vsizer = wx.BoxSizer(wx.VERTICAL)

        self.buttonSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.buttonSizer.Add(wx.Button(self, 101, "Open Frames"), 0)
        self.Bind(wx.EVT_BUTTON, self.open_frames, id=101)
        self.buttonSizer.Add(wx.Button(self, 102, "Set Plot Title"), 0)
        self.Bind(wx.EVT_BUTTON, self.changePlotTitle, id=102)
        self.buttonSizer.Add(wx.Button(self, 103, "Set Y Label"), 0)
        self.Bind(wx.EVT_BUTTON, self.changeYLabel, id=103)
        self.buttonSizer.Add(wx.Button(self, 104, "Set Y Min"), 0)
        self.Bind(wx.EVT_BUTTON, self.setYMin, id=104)
        self.buttonSizer.Add(wx.Button(self, 105, "Set Y Max"), 0)
        self.Bind(wx.EVT_BUTTON, self.setYMax, id=105)
        self.buttonSizer.Add(wx.Button(self, 106, "Set Font Size"), 0)
        self.Bind(wx.EVT_BUTTON, self.setFontSize, id=106)

        self.vsizer.Add(self.buttonSizer, 0)

        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.canvasSizer = wx.BoxSizer(wx.VERTICAL)

        self.canvasSizer.Add(self.canvas, 1, wx.ALL | wx.EXPAND | wx.CENTER, 0)
        self.canvasSizer.Add(self.toolbar, 0, wx.ALL | wx.EXPAND | wx.RIGHT, 0)

        self.logs = dict()

        self.logTree = CT.CustomTreeCtrl(self, agwStyle=wx.TR_DEFAULT_STYLE)
        self.logTreeRoot = self.logTree.AddRoot("Scrimmage Logs")
        self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.logTreeChecked)

        self.keySizer = wx.BoxSizer(wx.VERTICAL)
        self.keySizer.Add(wx.StaticText(self, -1, "Key:"))

        rightColumn = wx.BoxSizer(wx.VERTICAL)
        rightColumn.Add(self.keySizer, 4)

        self.sizer.Add(self.logTree, 2, wx.ALL | wx.EXPAND | wx.CENTER, 5)
        self.sizer.Add(self.canvasSizer, 8, wx.ALL | wx.EXPAND | wx.CENTER, 5)
        self.sizer.Add(rightColumn, 4, wx.ALL | wx.EXPAND | wx.TOP, 5)

        self.vsizer.Add(self.sizer, 20, wx.ALL | wx.EXPAND | wx.TOP, 5)
        self.SetSizer(self.vsizer)
        self.Fit()

        self.plottedItems = list()
        self.plottedButtons = dict()
        self.plottedColors = dict()
        self.closeButtonNum = 0
        self.closeLookup = dict()

        self.timeShift = dict()
        self.timeShiftSliders = dict()

        self.plotTitle = None
        self.ymin = None
        self.ymax = None
        self.ylabel = None

        self.flyingStates = set()
        self.flyingStatesToPlot = set()

    def OnPaint(self, event):
        self.canvas.draw()

    def open_frames(self, event):
        file_dialog = \
            wx.FileDialog(self, "Open Frames",
                          os.environ['HOME'] + '/swarm-log',
                          "", "*.bin;*.pickle",
                          wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)

        if file_dialog.ShowModal() == wx.ID_CANCEL:
            return

        fname = file_dialog.GetPath()
        ext = os.path.splitext(fname)[1]
        dir_name = os.path.dirname(fname)
        log_name = os.path.basename(dir_name)

        if ext == '.pickle':
            contacts_df = pd.read_pickle(fname)
        else:
            contacts_df = sc.frames2pandas(fname)
            contacts_df.to_pickle(os.path.join(dir_name, "frames.pickle"))

        self.logs[log_name] = contacts_df

        log_root = self.logTree.AppendItem(self.logTreeRoot, log_name)
        columns = contacts_df.columns
        for uav_id in contacts_df['id'].unique():
            uav_root = self.logTree.AppendItem(log_root, str(uav_id))
            for column in columns:
                self.logTree.AppendItem(uav_root, column, ct_type=1)

        self.plotChanged()

    def changePlotTitle(self, event):
        dlg = wx.TextEntryDialog(self, "New plot title", "Plot Title")
        if self.plotTitle is not None:
            dlg.SetValue(self.plotTitle)
        if dlg.ShowModal() == wx.ID_OK:
            self.plotTitle = dlg.GetValue()
            self.ax.set_title(self.plotTitle)
            self.figure.canvas.draw()
        dlg.Destroy()

    def changeYLabel(self, event):
        dlg = wx.TextEntryDialog(self, "New y axis label", "Y Label")
        if self.ylabel is not None:
            dlg.SetValue(self.ylabel)
        if dlg.ShowModal() == wx.ID_OK:
            self.ylabel = dlg.GetValue()
            self.ax.set_ylabel(self.ylabel)
            self.figure.canvas.draw()
        dlg.Destroy()

    def setYMin(self, event):
        dlg = wx.TextEntryDialog(self, "New minimum y value", "Y Min")
        ymin, ymax = self.ax.get_ylim()
        dlg.SetValue(str(ymin))
        if dlg.ShowModal() == wx.ID_OK:
            self.ymin = float(dlg.GetValue())
            self.ax.set_ylim(bottom=self.ymin)
            self.figure.canvas.draw()
        dlg.Destroy()

    def setYMax(self, event):
        dlg = wx.TextEntryDialog(self, "New maximum y value", "Y Max")
        ymin, ymax = self.ax.get_ylim()
        dlg.SetValue(str(ymax))
        if dlg.ShowModal() == wx.ID_OK:
            self.ymax = float(dlg.GetValue())
            self.ax.set_ylim(top=self.ymax)
            self.figure.canvas.draw()
        dlg.Destroy()

    def setFontSize(self, event):
        dlg = wx.TextEntryDialog(self, "New font size", "Font Size")
        s = mpl.rc_params()['font.size']
        dlg.SetValue(str(s))
        if dlg.ShowModal() == wx.ID_OK:
            s = float(dlg.GetValue())
            mpl.rcParams.update({'font.size': s})
            self.figure.canvas.draw()
        dlg.Destroy()

    def closeButton(self, event):
        t = self.closeLookup[event.GetId()]
        item, garbage = self.logTree.GetFirstChild(self.logTreeRoot)
        while item.IsOk():
            if self.logTree.GetItemText(item) == t[0]:
                for p in item.GetChildren():
                    if self.logTree.GetItemText(p) == t[1]:
                        for c in p.GetChildren():
                            if self.logTree.GetItemText(c) == t[2]:
                                c.Check(False)
                                self.logTree.RefreshSubtree(c)
                                break
                        break
                break
        self.plottedItems.remove(t)
        self.plotChanged()

    def logTreeChecked(self, event):
        item = event.GetItem()
        parent = item.GetParent()
        grandparent = parent.GetParent()

        meta_data = (
            self.logTree.GetItemText(grandparent),
            self.logTree.GetItemText(parent),
            self.logTree.GetItemText(item)
        )

        if item.IsChecked():
            self.plottedItems.append(meta_data)
            self.plottedButtons[meta_data] = self.closeButtonNum
            self.closeLookup[self.closeButtonNum] = meta_data
            self.closeButtonNum += 1
        else:
            self.plottedItems.remove(meta_data)
        self.plotChanged()

    def plotChanged(self):
        # clear figure in preparation of new/fewer plots
        self.figure.clf()
        self.ax = self.figure.add_subplot(111)

        # reset key
        self.keySizer.DeleteWindows()
        self.keySizer.Add(wx.StaticText(self, -1, "Key:"))

        for item in self.plottedItems:
            log_name, uav_id, stat = item
            df = self.logs[log_name]
            data = df[df['id'] == int(uav_id)][['time', stat]]
            l, = self.ax.plot(data['time'], data[stat])
            self.plottedColors[item] = l.get_color()
            s = wx.BoxSizer(wx.HORIZONTAL)
            r = wx.Panel(self, -1, size=(10, 30))
            r = wx.StaticText(self, -1, "  ")
            # kind of a pain to get the RGB color out of pyplot
            r.SetBackgroundColour(
                tuple(i * 255 for i in
                      mpl.colors.ColorConverter.colors[l.get_color()])
            )
            s.Add(r, 0)
            s.Add(wx.StaticText(self, -1, "%s - %s - %s" % item,
                  style=wx.ALIGN_LEFT), 4, wx.ALL | wx.EXPAND | wx.LEFT)
            s.Add(wx.Button(self, self.plottedButtons[item], 'Remove'), 0)
            self.Bind(wx.EVT_BUTTON, self.closeButton,
                      id=self.plottedButtons[item])
            self.keySizer.Add(s)
        self.keySizer.Layout()

        self.ax.set_xlabel("Time")
        self.ax.relim()
        self.ax.autoscale_view()

        if self.ymin is not None:
            self.ax.set_ylim(bottom=self.ymin)
        if self.ymax is not None:
            self.ax.set_ylim(top=self.ymax)
        if self.ylabel is not None:
            self.ax.set_ylabel(self.ylabel)
        if self.plotTitle is not None:
            self.ax.set_title(self.plotTitle)

        self.figure.autofmt_xdate()
        self.figure.canvas.draw()
        self.SetSizer(self.vsizer)


class App(wx.App):
    def OnInit(self):
        frame = CanvasFrame()
        frame.Show(True)
        return True

app = App(0)
app.MainLoop()
