#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Not titled yet
# GNU Radio version: 3.10.1.1

from packaging.version import Version as StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio.filter import firdes
import sip
from gnuradio import blocks
from gnuradio import filter
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore
import estop_epy_block_3 as epy_block_3  # embedded python block
import numpy as np
import osmosdr
import time



from gnuradio import qtgui

class estop(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Not titled yet", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Not titled yet")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "estop")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 0.1e6
        self.cov_time = cov_time = 50

        ##################################################
        # Blocks
        ##################################################
        self.tab = Qt.QTabWidget()
        self.tab_widget_0 = Qt.QWidget()
        self.tab_layout_0 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab_widget_0)
        self.tab_grid_layout_0 = Qt.QGridLayout()
        self.tab_layout_0.addLayout(self.tab_grid_layout_0)
        self.tab.addTab(self.tab_widget_0, 'Tab 0')
        self.tab_widget_1 = Qt.QWidget()
        self.tab_layout_1 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab_widget_1)
        self.tab_grid_layout_1 = Qt.QGridLayout()
        self.tab_layout_1.addLayout(self.tab_grid_layout_1)
        self.tab.addTab(self.tab_widget_1, 'Tab 1')
        self.tab_widget_2 = Qt.QWidget()
        self.tab_layout_2 = Qt.QBoxLayout(Qt.QBoxLayout.TopToBottom, self.tab_widget_2)
        self.tab_grid_layout_2 = Qt.QGridLayout()
        self.tab_layout_2.addLayout(self.tab_grid_layout_2)
        self.tab.addTab(self.tab_widget_2, 'Tab 2')
        self.top_layout.addWidget(self.tab)
        self._cov_time_range = Range(0, 100, 1, 50, 200)
        self._cov_time_win = RangeWidget(self._cov_time_range, self.set_cov_time, "'cov_time'", "counter_slider", int, QtCore.Qt.Horizontal)
        self.tab_layout_2.addWidget(self._cov_time_win)
        self.rtlsdr_source_0 = osmosdr.source(
            args="numchan=" + str(1) + " " + ""
        )
        self.rtlsdr_source_0.set_time_now(osmosdr.time_spec_t(time.time()), osmosdr.ALL_MBOARDS)
        self.rtlsdr_source_0.set_sample_rate(samp_rate)
        self.rtlsdr_source_0.set_center_freq(433.96e6, 0)
        self.rtlsdr_source_0.set_freq_corr(46, 0)
        self.rtlsdr_source_0.set_dc_offset_mode(0, 0)
        self.rtlsdr_source_0.set_iq_balance_mode(0, 0)
        self.rtlsdr_source_0.set_gain_mode(True, 0)
        self.rtlsdr_source_0.set_gain(20, 0)
        self.rtlsdr_source_0.set_if_gain(30, 0)
        self.rtlsdr_source_0.set_bb_gain(30, 0)
        self.rtlsdr_source_0.set_antenna('', 0)
        self.rtlsdr_source_0.set_bandwidth(0, 0)
        self.qtgui_time_sink_x_2 = qtgui.time_sink_f(
            1024, #size
            samp_rate, #samp_rate
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_2.set_update_time(0.10)
        self.qtgui_time_sink_x_2.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_2.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_2.enable_tags(True)
        self.qtgui_time_sink_x_2.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_2.enable_autoscale(False)
        self.qtgui_time_sink_x_2.enable_grid(False)
        self.qtgui_time_sink_x_2.enable_axis_labels(True)
        self.qtgui_time_sink_x_2.enable_control_panel(True)
        self.qtgui_time_sink_x_2.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [0, 3, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_2.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_2.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_2.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_2.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_2.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_2.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_2.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_2_win = sip.wrapinstance(self.qtgui_time_sink_x_2.qwidget(), Qt.QWidget)
        self.tab_layout_0.addWidget(self._qtgui_time_sink_x_2_win)
        self.low_pass_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                50e3,
                50e3,
                window.WIN_HAMMING,
                6.76))
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_fcf(1, np.arange(cov_time), 0, samp_rate)
        self.epy_block_3 = epy_block_3.blk()
        self.dc_blocker_xx_1 = filter.dc_blocker_cc(32, True)
        self.blocks_peak_detector_xb_0 = blocks.peak_detector_fb(0.025, 0.025, 1, 0.001)
        self.blocks_complex_to_mag_0 = blocks.complex_to_mag(1)
        self.blocks_complex_to_float_0 = blocks.complex_to_float(1)
        self.blocks_char_to_float_0 = blocks.char_to_float(1, 1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_char_to_float_0, 0), (self.epy_block_3, 0))
        self.connect((self.blocks_complex_to_float_0, 0), (self.blocks_peak_detector_xb_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.freq_xlating_fir_filter_xxx_0, 0))
        self.connect((self.blocks_complex_to_mag_0, 0), (self.qtgui_time_sink_x_2, 0))
        self.connect((self.blocks_peak_detector_xb_0, 0), (self.blocks_char_to_float_0, 0))
        self.connect((self.dc_blocker_xx_1, 0), (self.low_pass_filter_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.blocks_complex_to_float_0, 0))
        self.connect((self.low_pass_filter_0, 0), (self.blocks_complex_to_mag_0, 0))
        self.connect((self.rtlsdr_source_0, 0), (self.dc_blocker_xx_1, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "estop")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate, 50e3, 50e3, window.WIN_HAMMING, 6.76))
        self.qtgui_time_sink_x_2.set_samp_rate(self.samp_rate)
        self.rtlsdr_source_0.set_sample_rate(self.samp_rate)

    def get_cov_time(self):
        return self.cov_time

    def set_cov_time(self, cov_time):
        self.cov_time = cov_time
        self.freq_xlating_fir_filter_xxx_0.set_taps(np.arange(self.cov_time))




def main(top_block_cls=estop, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
