import threading
import queue

import json
import backtrader as bt
import backtrader.feeds as btfeeds
import backtrader.indicators as btind
import backtrader.utils.flushfile

import datetime


data_queue = queue.Queue()

class ROS2DataFeed(bt.feed.DataBase):
    params = (
        ("dataname", None),
        ("fromdate", datetime.date(2000, 1, 1)),
        ("todate", datetime.date(2050, 1, 1)),
        ("compression", 1),
        ("timeframe", bt.TimeFrame.Minutes),
        ("symbol", None),
    )

    def __init__(self, topic_name="test", *args, **kwargs):
        super().__init__(*args, **kwargs)

    def test(self):
        while True:
            global data_queue
            try:
                temp = data_queue.get(block=True, timeout=1)
            except queue.Empty:
                print("Empty")
                break
            print('Publishing: "%s"' % temp)
        

    def start(self):
        pass

    def stop(self):
        pass
    
    def _load(self):
        
        done = False
        global data_queue
        while not done:
            try:
                row = data_queue.get(block=True, timeout=1)

                json_val = json.loads(row)
                # print(row[0])
                self.lines.datetime[0] = bt.date2num(datetime.datetime.fromtimestamp(json_val["s"]/1000))
                self.lines.open[0] = json_val["o"]
                self.lines.high[0] = json_val["h"]
                self.lines.low[0] = json_val["l"]
                self.lines.close[0] = json_val["c"]
                self.lines.volume[0] = json_val["v"]
                self.lines.openinterest[0] = 0
                done = True
            except queue.Empty:
                return False
            except Exception as e:
                print(e)
                print(json_val)
                pass
        # print(self.lines.datetime[0])
        return True

class TestInd(bt.Indicator):
    lines = ('a', 'b')

    def __init__(self):
        self.lines.a = b = self.data.close - self.data.high
        self.lines.b = btind.SMA(b, period=20)


class St(bt.SignalStrategy):
    params = (
        ('datalines', False),
        ('lendetails', False),
    )

    def __init__(self):
        btind.SMA()
        btind.Stochastic()
        btind.RSI()
        btind.MACD()
        btind.CCI()
        TestInd().plotinfo.plot = False
        sma1, sma2 = bt.ind.SMA(period=10), bt.ind.SMA(period=30)
        crossover = bt.ind.CrossOver(sma1, sma2)
        self.signal_add(bt.SIGNAL_LONG, crossover)

    def next(self):
        if self.p.datalines:
            txt = ','.join(
                ['%04d' % len(self),
                 '%04d' % len(self.data0),
                 self.data.datetime.date(0).isoformat()]
            )

            print(txt)

    def loglendetails(self, msg):
        if self.p.lendetails:
            print(msg)

    def stop(self):
        super(St, self).stop()

        tlen = 0
        self.loglendetails('-- Evaluating Datas')
        for i, data in enumerate(self.datas):
            tdata = 0
            for line in data.lines:
                tdata += len(line.array)
                tline = len(line.array)

            tlen += tdata
            logtxt = '---- Data {} Total Cells {} - Cells per Line {}'
            self.loglendetails(logtxt.format(i, tdata, tline))

        self.loglendetails('-- Evaluating Indicators')
        for i, ind in enumerate(self.getindicators()):
            tlen += self.rindicator(ind, i, 0)

        self.loglendetails('-- Evaluating Observers')
        for i, obs in enumerate(self.getobservers()):
            tobs = 0
            for line in obs.lines:
                tobs += len(line.array)
                tline = len(line.array)

            tlen += tdata
            logtxt = '---- Observer {} Total Cells {} - Cells per Line {}'
            self.loglendetails(logtxt.format(i, tobs, tline))

        print('Total memory cells used: {}'.format(tlen))

    def rindicator(self, ind, i, deep):
        tind = 0
        for line in ind.lines:
            tind += len(line.array)
            tline = len(line.array)

        thisind = tind

        tsub = 0
        for j, sind in enumerate(ind.getindicators()):
            tsub += self.rindicator(sind, j, deep + 1)

        iname = ind.__class__.__name__.split('.')[-1]

        logtxt = '---- Indicator {}.{} {} Total Cells {} - Cells per line {}'
        self.loglendetails(logtxt.format(deep, i, iname, tind, tline))
        logtxt = '---- SubIndicators Total Cells {}'
        self.loglendetails(logtxt.format(deep, i, iname, tsub))

        return tind + tsub


def runstrat(topic_name):
    save = 1
    datalines = True
    plot = False
    lendetails = False

    cerebro = bt.Cerebro()
    cerebro.adddata(ROS2DataFeed(topic_name))
    cerebro.addstrategy(
        St, datalines=datalines, lendetails=lendetails)

    result = cerebro.run(runonce=False, exactbars=save)
    if plot:
        cerebro.plot(style='bar')
    print("End")

class BacktraderThread(threading.Thread):
    def __init__(self, topic_name="test"):
        super().__init__()
        self.topic_name = topic_name
    def run(self):
        runstrat(self.topic_name)