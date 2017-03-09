#!/usr/bin/env python  

#icons:
#echo
#write
#filter
#plot

# TODO:
# add button for plotting (make nice mouseover that explains what it expects)
# figure out how to plot...
# add some visualization to JS state (?)
# add checkboxes for gamepad settings
##
# add a toolbar to replace those buttons
# add baud,port,js(?) pulldowns, and write/echo JS checkboxes to toolbar
# replace JS multibyte commands with chr(val), and get arduino to read it
# add non-analog info about arduino to right col? eg: controlelr states, 

#notebook:
#plotter
#gl viewer
#
#serial console
#
#info:
#tty
#bud
#status lights for JS, com, *even Heli*:
#     -controller status
#
#  1 got back to where i was yesterday, but with new interface.  Next up is 
#  2 fixing the joystick->terminal calls to use a queue or something so 
#  3 that they're threadsafe.  Currently it bombs out on linux and mac.  
#  4 Another thing is to figure out whether it's bad that my mac seems to 
#  5 need to initialize pygame at the top, and why it needs gamepad 
#  6 declared after terminal in arducom



# Class wrapper for reading joystick values and passing messages over serial
# to an arduino running a helicopter controller


#
################################
#read joystick events into local variables with appropriate names
#
#after each event, either:
#	call function to print all values of each var
#	call function to write that (or all?) vars over serial
#	
#* will need to calibrate analog values for the PWM range
#	-left stick between middle and up gets discretized by 255
#	-right stick right sets 255,-255
#	-right stick left sets -255,255
#	-right stick up sets forward value between 0-255 and backward to 0
#	-right stick down does opposite...
#
################################
#GUI:
#LED to show if joystick is connected
#LED to show if arduino is connected
#table to show most recently written values from gamepad
#frame to show traces of analog values from whatever's being read (gyro, accelero, magneto)
#manual serial terminal

import wx
import wx.lib.plot as plot
import serial
import threading
import Queue
import pygame
import glob
import time
import numpy

#################### UTILITIES ###################
def rescale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def getPorts():
    if wx.Platform == "__WXMAC__":
        portchoices = glob.glob("/dev/tty.usb*")
    elif wx.Platform == "__WXGTK__":
        portchoices = glob.glob("/dev/ttyUSB*")
        
    if len(portchoices) == 0:
        portchoices = ["/dev/tty.usbserial-A800csDh"]
        
    return portchoices
##################################################



#----------------------------------------------------------------------
# Create an own event type, so that GUI updates can be delegated
# this is required as on some platforms only the main thread can
# access the GUI without crashing. wxMutexGuiEnter/wxMutexGuiLeave
# could be used too, but an event is more elegant.
SERIALRX = wx.NewEventType()
# bind to serial data receive events
EVT_SERIALRX = wx.PyEventBinder(SERIALRX, 0)

class SerialRxEvent(wx.PyCommandEvent):
    eventType = SERIALRX
    def __init__(self, windowID, data):
        wx.PyCommandEvent.__init__(self, self.eventType, windowID)
        self.data = data

    def Clone(self):
        self.__class__(self.GetId(), self.data)

#----------------------------------------------------------------------

ID_CLEAR        = wx.NewId()
ID_SAVEAS       = wx.NewId()
ID_SETTINGS     = wx.NewId()
ID_TERM         = wx.NewId()
ID_EXIT         = wx.NewId()

NEWLINE_CR      = 0
NEWLINE_LF      = 1
NEWLINE_CRLF    = 2

EVT_OPENPORT    = 0
EVT_CLOSEPORT   = 1
EVT_OPENJS      = 2
EVT_CLOSEJS     = 3
EVT_TERMSETTINGS= 4
EVT_SETBAUD     = 5
EVT_SETPORT     = 6
EVT_ECHOJS      = 7
EVT_WRITEJS     = 8
EVT_FILTER      = 9
EVT_PLOT        = 10

pygame.display.init() # mac needs these declared at the top.  fishy...

class ArduCom(wx.Frame):
    def __init__(self, *args, **kwds):
        wx.Frame.__init__(self, *args, **kwds)

        # Status bar
        self.CreateStatusBar()
        self.SetStatusText("ArduCom: A joystick serial commander for the Arduino")
        
         # Menu Bar
        self.menubar = wx.MenuBar()
        self.SetMenuBar(self.menubar)
        # File menu pulldown
        filemenu = wx.Menu()
        filemenu.Append(ID_CLEAR, "&Clear", "", wx.ITEM_NORMAL)
        filemenu.Append(ID_SAVEAS, "&Save Text As...", "", wx.ITEM_NORMAL)
        filemenu.AppendSeparator()
        filemenu.Append(ID_TERM, "&Terminal Settings...", "", wx.ITEM_NORMAL)
        filemenu.AppendSeparator()
        filemenu.Append(ID_EXIT, "&Exit", "", wx.ITEM_NORMAL)        
        self.menubar.Append(filemenu, "&File")
        # Other pulldowns?
        #end Menu Bar

        # Add content
        self.terminal = TerminalPanel(self)
        self.mygamepad = Gamepad(self) # this had to come after terminal for some reason.  Very fishy...
        self.write_queue = Queue.Queue(3) # this arg should limit the lag, but it doesn't
        
        self.tabs = MainNotebook(self, -1)
        self.tb = self.CreateToolBar(wx.TB_HORIZONTAL
            | wx.NO_BORDER
            | wx.TB_FLAT
            #| wx.TB_TEXT
            #| wx.TB_HORZ_LAYOUT
            )
        self.__configure_tb()

        self.__set_properties()
        self.__attach_events()          #register events
        self.__do_layout()

    def __configure_tb(self):
        tsize = (36,36) # native res is 48x48 for my custom icons
        self.tb.SetToolBitmapSize(tsize)

        serial_open_bmp = wx.Bitmap("icons/serial_open.png")        
        self.tb.AddLabelTool(EVT_OPENPORT, "Open Serial", serial_open_bmp, 
            shortHelp="Open Serial", longHelp="Opens serial port")
        serial_close_bmp = wx.Bitmap("icons/serial_close.png")
        self.tb.AddLabelTool(EVT_CLOSEPORT, "Close Serial", serial_close_bmp, 
            shortHelp="Close Serial", longHelp="Closes serial port")

        self.tb.AddSeparator()
        gamepad_open_bmp = wx.Bitmap("icons/gamepad_open.png")
        self.tb.AddLabelTool(EVT_OPENJS, "Open Gamepad", gamepad_open_bmp, 
            shortHelp="Open Gamepad", longHelp="Opens gamepad")
        gamepad_close_bmp = wx.Bitmap("icons/gamepad_close.png")
        self.tb.AddLabelTool(EVT_CLOSEJS, "Close Gamepad", gamepad_close_bmp, 
            shortHelp="Close Gamepad", longHelp="Closes gamepad")
        gamepad_echo_bmp = wx.Bitmap("icons/gamepad_echo.png")
        self.tb.AddCheckLabelTool(EVT_ECHOJS, "Joystick Echo", gamepad_echo_bmp,
            shortHelp="Echo Joystick commands to stdout (depress to turn off)")
        gamepad_write_bmp = wx.Bitmap("icons/gamepad_transmit.png")
        self.tb.AddCheckLabelTool(EVT_WRITEJS, "Joystick Transmit", gamepad_write_bmp,
            shortHelp="Transmit Joystick commands over serial port (depress to turn off)")
      
        self.tb.AddSeparator()
        info_bmp = wx.Bitmap("icons/serial_config.png")
        self.tb.AddLabelTool(EVT_TERMSETTINGS, "Terminal Info", info_bmp, 
            shortHelp="Term Info", longHelp="Opens terminal info dialog")

        self.tb.AddSeparator()        
        self.tb.AddControl(wx.Choice(self.tb, EVT_SETBAUD, (100,50), choices = ["9600", "115200"]))
        self.tb.AddControl(wx.Choice(self.tb, EVT_SETPORT, (100,50), choices = getPorts()))

        self.tb.AddSeparator()
        filter_bmp = wx.Bitmap("icons/filter.png")
        self.tb.AddCheckLabelTool(EVT_FILTER, "Filter Data", filter_bmp,
            shortHelp="Filter incoming data from serial port (depress to turn ON)")
        plot_bmp = wx.Bitmap("icons/plot.png")
        self.tb.AddCheckLabelTool(EVT_PLOT, "Plot Data", plot_bmp,
            shortHelp="Plot incoming data (depress to turn ON)")
            
        self.tb.Realize()

    def __set_properties(self):
        self.SetTitle("ArduCom")
        self.SetSize((1100, 750))

    def __attach_events(self):
        self.Bind(wx.EVT_MENU, self.OnExit, id = ID_EXIT)
        self.Bind(wx.EVT_MENU, self.OnTermSettings, id = ID_TERM)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_OPENPORT)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_CLOSEPORT)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_OPENJS)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_CLOSEJS)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_TERMSETTINGS)
        self.Bind(wx.EVT_COMBOBOX, self.OnToolClick, id = EVT_SETBAUD)
        self.Bind(wx.EVT_COMBOBOX, self.OnToolClick, id = EVT_SETPORT)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_ECHOJS)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_WRITEJS)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_FILTER)
        self.Bind(wx.EVT_TOOL, self.OnToolClick, id = EVT_PLOT)
        #self.Bind(wx.EVT_CLOSE, self.OnExit)
    
    def __do_layout(self):
        bottomsizer = wx.BoxSizer(wx.VERTICAL)
        topsizer = wx.BoxSizer(wx.HORIZONTAL)
        topsizer.Add(self.tabs,3,wx.EXPAND)
        bottomsizer.Add(topsizer, 2, wx.EXPAND)
        serialsizer = wx.StaticBoxSizer(wx.StaticBox(self, -1, "Serial Console"), wx.VERTICAL)
        serialsizer.Add(self.terminal, 1, wx.EXPAND)
        bottomsizer.Add(serialsizer, 1, wx.EXPAND)
        self.SetAutoLayout(True)
        self.SetSizer(bottomsizer)
        self.Layout()

    def OnToolClick(self, event):
        e = event.GetId()
        if e == EVT_OPENPORT:
            self.terminal.OpenPort()
        elif e == EVT_CLOSEPORT:
            self.terminal.OnClose(e)
        elif e == EVT_TERMSETTINGS:
            self.OnTermSettings(e)
        elif e == EVT_OPENJS:
            self.mygamepad.openJS()
        elif e == EVT_CLOSEJS:
            self.mygamepad.OnClose()
        elif e == EVT_SETBAUD:
            self.terminal.settings.baudrate = e.GetSelection()
        elif e == EVT_SETPORT:
            self.terminal.settings.port = e.GetSelection()
            print "set port to ", self.terminal.settings.port
        elif e == EVT_ECHOJS:
            self.mygamepad.settings.echo = not self.mygamepad.settings.echo
            print self.mygamepad.settings.echo
        elif e == EVT_WRITEJS:
            self.mygamepad.settings.write = not self.mygamepad.settings.write
            print self.mygamepad.settings.write
        elif e == EVT_FILTER:
            self.terminal.settings.runFilter = not self.terminal.settings.runFilter
            print "Filter = ", self.terminal.settings.runFilter
        elif e == EVT_PLOT:
            self.tabs.myplot.plotSerial = not self.tabs.myplot.plotSerial
            print "Plot = ", self.tabs.myplot.plotSerial
                                               
    def OnExit(self, event):
        """Menu point Exit"""
        self.terminal.OnClose(ID_EXIT)
        self.Destroy()

    def OnTermSettings(self, event):
        """Menu point Terminal Settings. Show the settings dialog
           with the current terminal settings"""
        dialog = TerminalSettingsDialog(None, -1, "", settings=self.terminal.settings)
        result = dialog.ShowModal()
        dialog.Destroy()        

class MainNotebook(wx.Notebook):
    def __init__(self, parent, id):
        wx.Notebook.__init__(self, parent, id, style=
                             wx.BK_DEFAULT
                             #wx.BK_TOP 
                             #wx.BK_BOTTOM
                             #wx.BK_LEFT
                             #wx.BK_RIGHT
                             # | wx.NB_MULTILINE
                             )
        self.myplot = PlotPanel(self)
        self.myplot.StartThread()
        self.AddPage(self.myplot, "My Plot")

        win = self.makeColorPanel(wx.CYAN)
        self.AddPage(win, "Cyan")

        # TODO: remove colorpanel crap and add a glcanvas
     
    def makeColorPanel(self, color):
        p = wx.Panel(self, -1)
        win = ColorPanel(p, color)
        p.win = win
        def OnCPSize(evt, win=win):
            win.SetPosition((0,0))
            win.SetSize(evt.GetSize())
        p.Bind(wx.EVT_SIZE, OnCPSize)
        return p

class ColorPanel(wx.Window):
    def __init__(self, parent, color):
        wx.Window.__init__(self, parent, -1, style = wx.SIMPLE_BORDER)
        self.SetBackgroundColour(color)
        if wx.Platform == '__WXGTK__':
            self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
            
class PlotPanel(plot.PlotCanvas):
    def __init__(self, parent):
        plot.PlotCanvas.__init__(self,parent)
        self.thread = None
        self.plotSerial = False             # Flag for whether to run plotdata()
        self.alive = threading.Event()
        self.plotQueue = Queue.Queue(30)   # The Queue that plotdata() reads from
#        self.xVals = range(0,30) #TODO make this a variable
#        self.yVals = [[0]*30]*5 #TODO for now assume we're getting 5 lines
#        self.xVals = numpy.array([range(0,50)])
        #self.yVals = numpy.array([zeros(50)])
        self.yVals = [0]*50

    def OnClose(self):
        """Called on application shutdown."""
        self.plotSerial = False
        self.StopThread()            # stop reader thread

    def StartThread(self):
        """Start the receiver thread"""        
        self.thread = threading.Thread(target=self.readQueue)
        self.thread.setDaemon(1)
        self.alive.set()
        self.thread.start()

    def StopThread(self):
        """Stop the receiver thread, wait util it's finished."""
        if self.thread is not None:
            self.alive.clear()          # clear alive event for thread
            self.myjoy.quit()            # cleanup
            self.thread.join()         # this thread never seems to die by itself
            self.thread = None
                
    def readQueue(self):
        while True:
            data = self.plotQueue.get()
            while self.plotQueue.qsize() > 5:
                self.plotQueue.get_nowait()
            print "qsize = ",self.plotQueue.qsize()
            if self.plotSerial:
                # format a numpy array with 1:200 and yvals
                # initialize as zeros and 1:200
                # on each get, push a value from the back (possible)
                #self.data.append(numpy.transpose([d,[len(d[0])]*len(d)])
                self.yVals.pop(0)
                self.yVals.append(int(data[0]))
    #            data = numpy.vstack((self.xVals,self.yVals))
                d=[]
                for i in range(0,50):
                    d.append((i,self.yVals[i]))
#                print d
                self.plotdata(d);
    #            self.plotdata([[1,2,3,4],[2,4,6,4]]);
    #            self.plotdata([(1,2), (2,3), (3,5), (4,6), (5,8), (6,8), (10,10)]);
                self.plotQueue.task_done()
#                print "got here"
               
    def plotdata(self, data):
        """ Waits on a plotter queue, and redraws whenever a new datapoint is posted"""
#        self.data = [(1,2), (2,3), (3,5), (4,6), (5,8), (6,8), (10,10)]
#        line = plot.PolyLine(self.data, legend='', colour='pink', width=1)
#        gc = plot.PlotGraphics([line], 'Line Graph', 'X Axis', 'Y Axis')
#        self.Draw(gc, xAxis= (0,15), yAxis= (0,15))
        line = plot.PolyLine(data, legend='', colour='pink', width=3)
        gc = plot.PlotGraphics([line], 'Line Graph', 'X Axis', 'Y Axis')
        self.Draw(gc)

class GamepadSetup:
    """Placeholder for various gamepad settings. Used to pass the
       options to the GamepadSettingsDialog."""
    def __init__(self):
        self.echo = False
        self.write = False

class Gamepad(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.parent = parent
        self.myjoy = None
        self.thread = None
        self.alive = threading.Event()
        self.settings = GamepadSetup() #placeholder for the settings

    def openJS(self):
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
                print "\nPlease connect a joystick and run again.\n"  
        else:
            self.joysticks = []
            print "\n%d joystick(s) detected." % pygame.joystick.get_count()
            for i in range(pygame.joystick.get_count()):  
                self.myjoy = pygame.joystick.Joystick(i)  
                self.myjoy.init()
                self.joysticks.append(self.myjoy)  
                print "Joystick %d: " % (i) + self.joysticks[i].get_name()  

            self.StartThread()

    def pollJSThread(self):
        while self.doPoll == True:  
            e = pygame.event.wait()
            if (e.type == pygame.JOYBUTTONDOWN or e.type == pygame.JOYAXISMOTION):
                self.handleJoyEvent(e)
                
    def OnClose(self):
        """Called on application shutdown."""
        self.doPoll = False
        pygame.event.post(pygame.event.Event(pygame.K_BREAK)) # hack to get while loop to break
        self.StopThread()            # stop reader thread

    def StartThread(self):
        """Start the receiver thread"""        
        self.thread = threading.Thread(target=self.pollJSThread)
        self.thread.setDaemon(1)
        self.alive.set()
        self.doPoll = True
        self.thread.start()

    def StopThread(self):
        """Stop the receiver thread, wait util it's finished."""
        if self.thread is not None:
            self.alive.clear()          # clear alive event for thread
            self.myjoy.quit()            # cleanup
            #self.thread.join()         # this thread never seems to die by itself
            self.thread = None

    def handleJoyEvent(self,e):
        if e.type == pygame.JOYAXISMOTION:
            cmd = ''
            val = 0
            #print e.axis, e.value
            if e.axis == 1: # left-vertical
                # srping load mode: write values in proportion to up-stick; ignore down-stick
                val = int(rescale(e.value,0,-1,0,255))
                val = max(val,0)
                cmd = '!' # thrust
            elif e.axis == 2: # right-horizontal
                val = int(rescale(e.value,-1,1,0,100))
                cmd = '#' # yaw (same as $)
            elif e.axis == 3: # right-vertical
                val = int(rescale(e.value,-1,1,-250,250))
                if val > 0:
                    cmd = '^' # forward
                else:
                    cmd = '%' # backward
                    val = -val
            if e.axis != 0:
                if self.settings.echo:
                    print e.axis, e.value, "cmd: ", 'm'+str(val)+cmd+'m'
                if self.settings.write:
                    if self.parent.terminal.alive.isSet(): # don't fill the queue unless someone is clearing it
                        self.parent.write_queue.put('m'+str(val)+cmd+'m') # TODO: convert this to 3 byte version (currently 6)
        elif e.type == pygame.JOYBUTTONDOWN:  
            cmd = ' '
            if e.button == 0:
                cmd = 'v' # verbose
            if e.button == 1:
                cmd = 'm51#m' # toggle all controllers
            if e.button == 2:
                cmd = 'y' # yaw controller
            if e.button == 3:
                cmd = 'p' # pitch controller
            if e.button == 4:
                cmd = ',' # pgain--
            if e.button == 5:
                cmd = '.' # pgain++
            if e.button == 6:
                cmd = '<' # dgain--
            if e.button == 7:
                cmd = '>' # dgain++
            if e.button == 8:
                cmd = 'l' # log data (print analogRead vals to stdout)
            if e.button == 9:
                cmd = ' ' # reset
            if self.settings.echo:
                print e.button, "cmd: ",cmd
            if self.settings.write:
                self.parent.write_queue.put(cmd)
                                
class TerminalPanel(wx.Panel):
    """Simple terminal program for wxPython"""
    
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        self.parent = parent
        self.serial = serial.Serial()
        self.serial.timeout = 0.5           # make sure that the alive event can be checked from time to time
        self.settings = TerminalSetup()     # placeholder for the settings
        self.dataMode = False               # currently building a data packet (see dataFilter)
        self.skip_for_newline = False       # clumsy way to avoid newlines between data reads
        self.dataBuf = []                   # container for a single data packet
        self.textBuf = []                   # container for non-data text
        self.readthread = None
        self.writethread = None
        self.alive = threading.Event()
        self.text_ctrl_output = wx.TextCtrl(self, -1, "", style=wx.TE_MULTILINE|wx.TE_READONLY)
        self.__attach_events()
        self.__do_layout()
    
    def __attach_events(self):
        self.Bind(EVT_SERIALRX, self.OnSerialRead)
        self.Bind(wx.EVT_MENU, self.OnClear, id = ID_CLEAR)
        self.Bind(wx.EVT_MENU, self.OnSaveAs, id = ID_SAVEAS)
        self.text_ctrl_output.Bind(wx.EVT_CHAR, self.OnKey)
        
    def __do_layout(self):
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_1.Add(self.text_ctrl_output, 1, wx.EXPAND, 0)
        self.SetAutoLayout(1)
        self.SetSizer(sizer_1)
        self.Layout()
        
    def OnClose(self, event):
        """Called on application shutdown."""
        self.StopThread()               #stop reader thread
        self.serial.close()             #cleanup

    def OnSaveAs(self, event):
        """Save contents of output window."""
        filename = None
        dlg = wx.FileDialog(None, "Save Text As...", ".", "", "Text File|*.txt|All Files|*",  wx.SAVE)
        if dlg.ShowModal() ==  wx.ID_OK:
            filename = dlg.GetPath()
        dlg.Destroy()
        
        if filename is not None:
            f = file(filename, 'w')
            text = self.text_ctrl_output.GetValue()
            if type(text) == unicode:
                text = text.encode("latin1")    #hm, is that a good asumption?
            f.write(text)
            f.close()
    
    def OnClear(self, event):
        """Clear contents of output window."""
        self.text_ctrl_output.Clear()
                
    def StartThread(self):
        """Start the receiver thread"""        
        self.readthread = threading.Thread(target=self.ComPortThread)
        self.readthread.setDaemon(1)

        self.writethread = threading.Thread(target=self. serialWriter)
        self.writethread.setDaemon(1)
        
        self.alive.set()
        self.readthread.start()
        self.writethread.start()

    def StopThread(self):
        """Stop the receiver thread, wait util it's finished."""
        if self.readthread is not None:
            self.alive.clear()          #clear alive event for thread
            self.readthread.join()          #wait until thread has finished
            #self.writethread.join()          #wait until thread has finished
            self.readthread = None
            #self.writethread = None

    def OpenPort(self):
        """Just initialize the port"""
        self.serial.baudrate = self.settings.baudrate
        self.serial.setPort(self.settings.port)
        print "Opening port: ", self.serial.port, " (", self.serial.baudrate, ")"
        try:
            self.serial.open()
            self.StartThread()
            # this title string should be passed instead to a staticboxsizer for htis panel
            # self.SetTitle("Serial Terminal on %s [%s, %s%s%s%s%s]" % (
            #     self.serial.portstr,
            #     self.serial.baudrate,
            #     self.serial.bytesize,
            #     self.serial.parity,
            #     self.serial.stopbits,
            #     self.serial.rtscts and ' RTS/CTS' or '',
            #     self.serial.xonxoff and ' Xon/Xoff' or '',
            #     )
            #)
            if not self.alive.isSet():
                print "closing.."
                self.Close()
        except serial.SerialException, e:
            dlg = wx.MessageDialog(None, str(e), "Serial Port Error", wx.OK | wx.ICON_ERROR)
            dlg.ShowModal()
            dlg.Destroy()
            
    def serialWriter(self):
        """Only this function gets to issue the serial.write call, since output
           to this port can potentially come from more than one source (kbd, joy).
           Reads a queue owned by arducom and executes commands in whatever order 
           they arrive.  *Python supports priority queues, which we could use to 
           force console commands over joy commands, but that hardly seems necessary"""
        if self.alive.isSet(): 
            while True:
                cmd = self.parent.write_queue.get()
                if self.settings.echo:          #do echo if needed
                    self.text_ctrl_output.WriteText(cmd)
                self.serial.write(cmd)
            
    def OnKey(self, event):
        """Key event handler. if the key is in the ASCII range, write it to the serial port.
           Newline handling and local echo is also done here."""
        if self.alive.isSet(): 
            code = event.GetKeyCode()
            if code < 256:                          #is it printable?
                if code == 13:                      #is it a newline? (check for CR which is the RETURN key)
                    if self.settings.echo:          #do echo if needed
                        self.text_ctrl_output.AppendText('\n')
                    cmd = ''
                    if self.settings.newline == NEWLINE_CR:
                        cmd = '\r'
                    elif self.settings.newline == NEWLINE_LF:
                        cmd = '\n'
                    elif self.settings.newline == NEWLINE_CRLF:
                        cmd = '\r\n'
                    self.parent.write_queue.put(cmd)
                else:
                    char = chr(code)
                    self.parent.write_queue.put(char)
            else:
                print "Extra Key:", code
        #else:
            #print "Serial Terminal not open"

    def ComPortThread(self):
        """Thread that handles the incomming traffic. Does the basic input
           transformation (newlines) and generates an SerialRxEvent"""
        while self.alive.isSet():               #loop while alive event is true
            text = self.serial.read(1)          #read one, with timout
            if text:                            #check if not timeout
                n = self.serial.inWaiting()     #look if there is more to read
                if n:
                    text = text + self.serial.read(n) #get it
                #newline transformation
                if self.settings.newline == NEWLINE_CR:
                    text = text.replace('\r', '\n')
                elif self.settings.newline == NEWLINE_LF:
                    pass
                elif self.settings.newline == NEWLINE_CRLF:
                    text = text.replace('\r\n', '\n')
                event = SerialRxEvent(self.GetId(), text)
                self.GetEventHandler().AddPendingEvent(event)
                
    def OnSerialRead(self, event):
        """Handle input from the serial port."""
        text = event.data   # Does this get multiple characters at once???
        #print text
        if self.settings.unprintable:
            # Substitute in ascii value in <> delimiters if character is unprintable (ord(c)<32)
            text = ''.join([(c >= ' ') and c or '<%d>' % ord(c)  for c in text])
        if self.settings.runFilter:
            filtered = self.dataFilter(text)
            del self.textBuf[:] # must clear this buffer here
            self.text_ctrl_output.AppendText(filtered)
        else:
            self.text_ctrl_output.AppendText(text)
                
    def dataFilter(self, text):
        """ Processes incoming serial data to extract analog data. 
            Incoming data is indicated by surrounding it with '[]' delimiters.
            This function is mainly usefull for passing parsed data to a plotter
            or openGL"""
        
        for t in text:
            # switch modes as necessary
            if self.skip_for_newline:
                self.skip_for_newline = False
                continue
            if t == '[':
                self.dataMode = True
            elif t == ']':
                self.dataMode = False
                # on closing packet, send as necessary
                if self.parent.tabs.myplot.plotSerial:
                    datastring = ''.join(self.dataBuf)
                    packet = datastring.split(' ')
                    #print "packet = ",packet
                    self.parent.tabs.myplot.plotQueue.put(packet)
                
                # Reqest skip of next iteration, which we assume is an undesirable newline
                self.skip_for_newline = True    
                # cleanup
                del self.dataBuf[:]
            else:
                # Append to appropriate list
                if self.dataMode:
                    self.dataBuf.append(t)
                else:
                    self.textBuf.append(t)
            
        # return only the non-data portion
        return ''.join(self.textBuf)
            
# end of class TerminalPanel

class TerminalSetup:
    """Placeholder for various terminal settings. Used to pass the
       options to the TerminalSettingsDialog."""
    def __init__(self):
        self.echo = False
        self.unprintable = False
        self.newline = NEWLINE_CRLF
        self.baudrate = 9600          # default: arduino over USB
        self.port = getPorts()[0]    # default: first USB tty on linux
        self.runFilter = False

class TerminalSettingsDialog(wx.Dialog):
    """Simple dialog with common terminal settings like echo, newline mode."""
    
    def __init__(self, *args, **kwds):
        self.settings = kwds['settings']
        del kwds['settings']
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        
        # Sizers must be declared before controls in OS X!
        mainsizer = wx.BoxSizer(wx.VERTICAL)
        baudsizer = wx.BoxSizer(wx.HORIZONTAL)
        portsizer = wx.BoxSizer(wx.HORIZONTAL)
        buttonsizer = wx.BoxSizer(wx.HORIZONTAL)
        formatsizer = wx.BoxSizer(wx.VERTICAL)
        
        # The dialog controls
        self.checkbox_echo = wx.CheckBox(self, -1, "Local Echo")
        self.checkbox_unprintable = wx.CheckBox(self, -1, "Show unprintable characters")
        self.radio_box_newline = wx.RadioBox(self, -1, "Newline Handling", 
            choices=["CR only", "LF only", "CR+LF"], majorDimension=0, style=wx.RA_SPECIFY_ROWS)
        self.button_ok = wx.Button(self, -1, "OK")
        self.button_cancel = wx.Button(self, -1, "Cancel")
        self.baudPulldown = wx.Choice(self, -1, (100,50), choices = ["9600", "115200"])
        self.portPulldown = wx.Choice(self, -1, (100,50), choices = getPorts())

        # Set properties
        self.SetTitle("Terminal Settings")
        self.radio_box_newline.SetSelection(0)
        self.button_ok.SetDefault()
        self.checkbox_echo.SetValue(self.settings.echo)
        self.checkbox_unprintable.SetValue(self.settings.unprintable)
        self.radio_box_newline.SetSelection(self.settings.newline)

        # Do Layout
        baudsizer.Add(wx.StaticText(self, -1, "Baudrate: ", (60,20)), 0, wx.TOP, 8)
        baudsizer.Add(self.baudPulldown, -1, wx.ALL, 4)
        portsizer.Add(wx.StaticText(self, -1, "Port: ", (60,20)), 0, wx.TOP, 8)
        portsizer.Add(self.portPulldown, -1, wx.ALL, 4)
        buttonsizer.Add(self.button_ok, 0, wx.ALL, 2)
        buttonsizer.Add(self.button_cancel, 0, wx.ALL, 2)
        mainsizer.Add(baudsizer, 0, wx.ALL, 4)
        mainsizer.Add(portsizer, 0, wx.ALL, 4)
        formatsizer.Add(self.checkbox_echo, 0, wx.ALL, 4)
        formatsizer.Add(self.checkbox_unprintable, 0, wx.ALL, 4)
        formatsizer.Add(self.radio_box_newline, 0, wx.ALL | wx.ALIGN_CENTER, 4)
        mainsizer.Add(formatsizer, 0, wx.ALL | wx.ALIGN_CENTER,1)
        mainsizer.Add(buttonsizer, 0, wx.ALL | wx.ALIGN_CENTER,1)
        self.SetSizer(mainsizer)
        self.SetAutoLayout(1)
        mainsizer.Fit(self)
        mainsizer.SetSizeHints(self)
        self.Layout()
        self.CenterOnScreen()

        self.Bind(wx.EVT_BUTTON, self.OnOK, id = self.button_ok.GetId())
        self.Bind(wx.EVT_BUTTON, self.OnCancel, id = self.button_cancel.GetId())
        self.Bind(wx.EVT_CHOICE, self.BaudChoice, self.baudPulldown)
#    def __set_properties(self):
#        self.SetTitle("Terminal Settings")
#        self.radio_box_newline.SetSelection(0)
#        self.button_ok.SetDefault()
#
#    def __do_layout(self):
#        mainsizer = wx.BoxSizer(wx.VERTICAL)
#        baudsizer = wx.BoxSizer(wx.HORIZONTAL)
#        portsizer = wx.BoxSizer(wx.HORIZONTAL)
#        buttonsizer = wx.BoxSizer(wx.HORIZONTAL)
#        formatsizer = wx.BoxSizer(wx.VERTICAL)
#        
#        baudsizer.Add(wx.StaticText(self, -1, "Baudrate: ", (60,20)), 0, wx.TOP, 8)
#        baudsizer.Add(self.baudPulldown, -1, wx.ALL, 4)
#        portsizer.Add(wx.StaticText(self, -1, "Port: ", (60,20)), 0, wx.TOP, 8)
#        portsizer.Add(self.portPulldown, -1, wx.ALL, 4)
#
#        buttonsizer.Add(self.button_ok, 0, wx.ALL, 2)
#        buttonsizer.Add(self.button_cancel, 0, wx.ALL, 2)
#        mainsizer.Add(baudsizer, 0, wx.ALL, 4)
#        mainsizer.Add(portsizer, 0, wx.ALL, 4)
#        formatsizer.Add(self.checkbox_echo, 0, wx.ALL, 4)
#        formatsizer.Add(self.checkbox_unprintable, 0, wx.ALL, 4)
#        formatsizer.Add(self.radio_box_newline, 0, wx.ALL | wx.ALIGN_CENTER, 4)
#        mainsizer.Add(formatsizer, 0, wx.ALL | wx.ALIGN_CENTER,1)
#        mainsizer.Add(buttonsizer, 0, wx.ALL | wx.ALIGN_CENTER,1)
#
#        self.SetSizer(mainsizer)
#        self.SetAutoLayout(1)
#        mainsizer.Fit(self)
#        mainsizer.SetSizeHints(self)
#        self.Layout()
#        self.CenterOnScreen()
#
#    def __attach_events(self):
#        self.Bind(wx.EVT_BUTTON, self.OnOK, id = self.button_ok.GetId())
#        self.Bind(wx.EVT_BUTTON, self.OnCancel, id = self.button_cancel.GetId())
#        self.Bind(wx.EVT_CHOICE, self.BaudChoice, self.baudPulldown)
    
    def BaudChoice(self, event):
        print "set baudrate to ", event.GetString()

    def OnOK(self, events):
        """Update data wil new values and close dialog."""
        self.settings.echo = self.checkbox_echo.GetValue()
        self.settings.unprintable = self.checkbox_unprintable.GetValue()
        self.settings.newline = self.radio_box_newline.GetSelection()
        self.settings.baudrate = self.baudPulldown.GetSelection()
        self.settings.port = self.portPulldown.GetSelection()
        self.EndModal(wx.ID_OK)
    
    def OnCancel(self, events):
        """Do not update data but close dialog."""
        self.EndModal(wx.ID_CANCEL)

 #end of class TerminalSettingsDialog 

class MyApp(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        mainFrame = ArduCom(None, -1, "")                                               
        self.SetTopWindow(mainFrame)
        mainFrame.Show(1)
        return 1

# end of class MyApp

if __name__ == "__main__":
    app = MyApp(0)
    app.MainLoop()

