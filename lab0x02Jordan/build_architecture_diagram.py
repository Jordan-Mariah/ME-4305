"""Generate an editable file/object connection diagram (not an FSM)."""
from pathlib import Path
import xml.etree.ElementTree as ET


def build():
    document = ET.Element('mxfile', host='app.diagrams.net')
    page = ET.SubElement(document, 'diagram', id='architecture', name='File and object connections')
    graph = ET.SubElement(page, 'mxGraphModel', page='1', pageWidth='1200', pageHeight='1200', grid='1')
    root = ET.SubElement(graph, 'root')
    ET.SubElement(root, 'mxCell', id='0')
    ET.SubElement(root, 'mxCell', id='1', parent='0')

    def box(key, label, x, y, w, h, color='#dae8fc'):
        cell = ET.SubElement(root, 'mxCell', id=key, value=label, vertex='1', parent='1',
                             style=f'rounded=1;html=1;whiteSpace=wrap;fillColor={color};'
                             'strokeColor=#555555;fontSize=17;spacing=12;')
        ET.SubElement(cell, 'mxGeometry', x=str(x), y=str(y), width=str(w), height=str(h), **{'as': 'geometry'})

    def arrow(key, source, target, label, extra=''):
        cell = ET.SubElement(root, 'mxCell', id=key, source=source, target=target, value=label,
                             edge='1', parent='1', style='edgeStyle=orthogonalEdgeStyle;rounded=1;'
                             'html=1;endArrow=block;endFill=1;fontSize=14;'
                             'labelBackgroundColor=#ffffff;strokeWidth=2;' + extra)
        ET.SubElement(cell, 'mxGeometry', relative='1', **{'as': 'geometry'})

    box('title', '<b>LAB 0x02 | HOW THE FILES CONNECT</b><br>Arrows show construction, calls, or hardware access; these boxes are modules, not FSM states.',
        40, 20, 1120, 90, '#ffffff')
    box('main', '<b>HIGH LEVEL: main.py</b><br>Imports Motor, Encoder, TaskMotor<br>Configures pins, timers, USB and supply voltage<br>Creates objects; calls task.run() repeatedly',
        300, 150, 600, 140)
    box('task', '<b>MIDDLE LEVEL: taskmotor.py</b><br>TaskMotor(motor, encoder, serial, supply_voltage)<br>Owns FSM, trial sweep, timing, buffers and CSV<br>Only task operates selected drivers after setup',
        300, 400, 600, 140, '#d5e8d4')
    box('alias', '<b>LOW LEVEL: motor.py</b><br>from driver import Motor<br>Import name only; same Motor class',
        50, 650, 420, 100, '#fff2cc')
    box('driver', '<b>LOW LEVEL: driver.py</b><br>Motor: enable(), set_effort(), disable()<br>Controls PWM, direction and sleep pins',
        50, 830, 420, 110, '#fff2cc')
    box('encoder', '<b>LOW LEVEL: encoder.py</b><br>Encoder: update(), zero()<br>get_position(), get_velocity()<br>Reads timer counts and computes velocity',
        670, 800, 450, 140, '#fff2cc')
    box('hardware', '<b>LEFT WHEEL HARDWARE</b><br>Motor: TIM4 / PB6 PWM, PB5 DIR, PA10 sleep<br>Encoder: TIM2 / PA0 and PA1 quadrature inputs',
        230, 1040, 740, 100, '#eeeeee')
    box('usb', '<b>USB / PuTTY</b><br>CSV after each trial', 940, 410, 220, 110, '#f8cecc')
    arrow('construct', 'main', 'task', 'Pass driver objects into task<br>while not task.done: task.run()')
    arrow('imports', 'main', 'alias', 'Import Motor;<br>construct motor',
          'dashed=1;exitX=0;exitY=0.5;entryX=0;entryY=0;')
    arrow('alias-link', 'alias', 'driver', 'Re-exports Motor class', 'dashed=1;')
    arrow('drive', 'task', 'driver', 'motor.enable()<br>motor.set_effort(u)<br>motor.disable()',
          'exitX=0.25;exitY=1;entryX=1;entryY=0.5;')
    arrow('read', 'task', 'encoder', 'encoder.update()<br>read position / velocity',
          'exitX=0.85;exitY=1;entryX=0.5;entryY=0;')
    arrow('serial', 'task', 'usb', 'serial.send()<br>timeout=0')
    arrow('power', 'driver', 'hardware', 'PWM / DIR / sleep')
    arrow('counts', 'encoder', 'hardware', 'Read quadrature counter')
    box('note', 'main.py also imports and constructs Encoder.<br>The two drivers do not call each other.<br>Solid arrows: calls/access. Dashed: import relationship.',
        600, 620, 550, 105, '#ffffff')
    path = Path(__file__).with_name('lab0x02_architecture.drawio')
    ET.indent(document, space='  ')
    ET.ElementTree(document).write(path, encoding='utf-8', xml_declaration=True)
    print(path)


if __name__ == '__main__':
    build()
