
"""Configure P trials, save complete response data, and plot velocity/voltage."""
#automate the collection of motor responses
#Can be run on PC after the manual firmware testing
#** Install dependencies with: python -m pip install pyserial matplotlib
#First must configure the serial ports
import csv #added
import math #added
from datetime import datetime
from pathlib import Path #added
from time import monotonic

# Serial-port settings. Change SERIAL_PORT to the computer's com port
SERIAL_PORT = "COM6"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# Single-character commands are sent as soon as they are chosen; they do not
# need a line ending. Students will extend this into the duty-cycle and
# two-motor command sequence. A multicharacter numeric value does need the line
# ending expected by the firmware so that it knows when entry is complete.

NUMERIC_LINE_ENDING = "\r" #changed

#Here we test duty cycle by reeplacing the starter's single START_COMMAND with two motor/duty pairs
#For tests we now have three different proportional gain trials 
#tune these
#now have a wheel 
RUN_MODE = "wheel"
LINE_BASE_SPEED = 2.0
LINE_KP = 0.1
LINE_DURATION_S = 2.0
LINE_FIRMWARE_READY = False

TESTS = (("left", 5.0, 0.7),("right", 5.0, 0.7))
# Stop waiting and report an error instead of hanging forever if the firmware
# does not respond or stops transmitting partway through a dataset.
# monotonic() returns a steadily increasing time in seconds. Its starting value
# is arbitrary, so it is not used as a date or time of day. Subtracting an old
# reading from a new reading gives the elapsed time without being affected if
# the computer's clock is adjusted while the program is running.
#FIRST_RESPONSE_TIMEOUT covers command acknowledgements and beginning marker.
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0
# The firmware should print this marker on its own line after the last data row.
# Using an explicit marker is more reliable than assuming that a quiet serial
# port means the dataset is complete.
END_MARKER = "End of data:" #* Outcome follows the colon.

#Here match the firware headers
HEADERS = ["time_us", "setpoint_rad_s", "velocity_rad_s", "limited_voltage_V"] #* MCU columns.
#organize the plots ouputs
OUTPUT_DIRECTORY = Path(__file__).resolve().parent / "results" #added


#turn serial bytes into computer into lines of text
class LineReader: #added
    #store serial conect and make empty byte buffer
    def __init__(self, serial): #added
        self.serial = serial #added
        #contructs mutable byte aray
        self.pending = bytearray() #added

    #recieves bytes until can make a full line then converts bytes to text
    def read(self, deadline): #added
        while monotonic() < deadline: #added- while clock less than dealine
            #search the buffered byte for a newline
            newline = self.pending.find(b"\n") #added
            if newline >= 0: #added
                #put everything before newline in raw
                raw_line = self.pending[:newline] #added
                #remove the bytes and newline characters from the buffer-added
                del self.pending[:newline + 1] 
                #convert bytes to pythons string, sub replacement char for incalid utf-8, and
                #remove surounding whitespace
                return raw_line.decode("utf-8", errors="replace").strip() #added
            # readline() returns b"" when its short serial timeout expires. Keep
            # polling until the longer application timeout has also expired.
            self.pending.extend(self.serial.readline()) #added
        raise TimeoutError(
            "Timed out waiting for a complete firmware line; partial bytes: {!r}".format(bytes(self.pending))) #changed

#wait for firmware acknowledgements
def wait_for(reader, expected): #added
    #figureout firware reply missing
    print(f"Waiting for: {expected!r}") #added
    #calc for when to stop waiting
    deadline = monotonic() + FIRST_RESPONSE_TIMEOUT #added
    while True: #added
        #asks reader for a fulll line of text
        try: #added
            line = reader.read(deadline) #added
        except TimeoutError as error: #added
            raise TimeoutError(f"Expected {expected!r}: {error}") from error #added
        print(line) #added
        #does recieved string match expected
        if line == expected: #added
            return line #acknowledgment
        #check for firmwareerrors
        if "out of range" in line or line.startswith(("Invalid input", "Motor test busy", "Test rejected")): #added
            raise RuntimeError(line) #added

#had copilot help with the structuring of the conditionalls uin this function
#to handle all of the the possible collection failure modes
def collect_dataset(reader, motor, setpoint, kp, filename):
    #save partial data if collection fails, and raise an error to stop the program
    deadline = monotonic() + FIRST_RESPONSE_TIMEOUT
    while True:
        metadata = reader.read(deadline)
        if metadata.startswith("Begin data: "):
            break
        if metadata.startswith(("Motor test busy", "Invalid input")):
            raise RuntimeError(metadata)
    config = dict(field.split("=", 1) for field in metadata[len("Begin data: "):].split(","))
    rows = []
    #Write as samples arrive. This way incomplete/failed runs retain their data.
    with filename.open("x", encoding="utf-8", newline="") as stream:
        stream.write("# " + metadata + "\n")
        writer = csv.writer(stream)
        writer.writerow(HEADERS)
        try:
            if (config.get("motor") != motor or config.get("mode") != "P"
                    or float(config["setpoint_rad_s"]) != setpoint or float(config["kp"]) != kp):
                raise ValueError("Firmware configuration does not match requested trial")
            wait_for(reader, ",".join(HEADERS))
            deadline = monotonic() + BETWEEN_LINES_TIMEOUT
            while True:
                line = reader.read(deadline)
                if line.startswith(END_MARKER):
                    stream.write("# " + line + "\n")
                    status = line.split(":", 1)[1].strip()
                    if status != "completed":
                        raise RuntimeError("Test outcome: " + status)
                    if not rows:
                        raise ValueError("Dataset contains no samples")
                    return metadata, config, rows
                values = [float(field) for field in line.split(",")]
                if len(values) != len(HEADERS) or not all(math.isfinite(v) for v in values):
                    raise ValueError("Malformed response row: " + line)
                if values[0] < 0 or (rows and values[0] <= rows[-1][0]):
                    raise ValueError("Timestamps must increase")
                writer.writerow(values)
                rows.append(values)
                deadline = monotonic() + BETWEEN_LINES_TIMEOUT
        except (Exception, KeyboardInterrupt) as error:
            stream.write("# Collection incomplete: {}\n".format(error))
            raise

#save the measurements as plots with matplotlib
def save_plot(filename, motor, setpoint, kp, config, rows):
    from matplotlib import pyplot
    # The measured velocity/setpoint are plotted together, and voltage on separate axes.
    figure, axes = pyplot.subplots(2, 1, sharex=True)
    try:
        times = [row[0] / 1_000_000 for row in rows]
        axes[0].plot(times, [row[2] for row in rows], label="Measured velocity")
        axes[0].plot(times, [row[1] for row in rows], "--", label="Setpoint")
        axes[0].set_ylabel("Velocity [rad/s]")
        axes[0].legend()
        axes[1].plot(times, [row[3] for row in rows])
        axes[1].set_ylabel("Limited voltage [V]")
        axes[1].set_xlabel("Elapsed time [s]")
        period = config.get("motor_period_ms", "unreported")
        axes[0].set_title(f"{motor.title()} P control: Kp={kp:g}, setpoint={setpoint:g} rad/s, period={period} ms")
        for axis in axes:
            axis.grid(True)
        figure.tight_layout()
        #save plot without overwriting current file
        with filename.open("xb") as output:
            figure.savefig(output, format="png", dpi=200)
    finally:
        pyplot.close(figure)


def configure_number(ser, reader, command, prompt, value):
    """for sending v and p and sends numbers terminated by carriage return"""
    ser.write(command)
    wait_for(reader, prompt)
    ser.write((str(value) + NUMERIC_LINE_ENDING).encode("ascii"))
    wait_for(reader, f"Value set to {value}")


#uses the serial port to run the left and right tests together
def main():
    #import pyserial class and serial exception class
    from serial import Serial, SerialException
    # Validate trials against with UI before opening the port
    #* just one trial or a sequence of trials.
    if not TESTS:
        raise ValueError("Configure at least one trial")
    for motor, setpoint, kp in TESTS:
        if motor not in ("left", "right"):
            raise ValueError("Choose left or right motor")
        if not math.isfinite(setpoint) or abs(setpoint) > 1000 or not math.isfinite(kp) or not 0 <= kp <= 1000:
            raise ValueError("Trial configuration is outside UI limits")
    # General results dirrectory   
    OUTPUT_DIRECTORY.mkdir(exist_ok=True) #changed
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f") #changed
    run_dir = OUTPUT_DIRECTORY / timestamp 
    run_dir.mkdir() 
    try: #timeout limits the serial read time, write_timeout limits write wait, with closes port
        with Serial(SERIAL_PORT, baudrate=BAUDRATE, #changed
                    timeout=SERIAL_TIMEOUT, write_timeout=2.0) as ser: #changed
            ser.reset_input_buffer()#disgard the unread serial all together
            reader = LineReader(ser) #added-call line reader class to assemble text lines
            try:
                #trial so dupes dont overwrite.
                for trial, (motor, selected_setpoint, selected_kp) in enumerate(TESTS, start=1):
                    setpoint, kp = float(selected_setpoint), float(selected_kp)
                    configure_number(ser, reader, b"v", "Enter velocity setpoint [rad/s]:", setpoint)
                    configure_number(ser, reader, b"p", "Enter Kp [V/(rad/s)]:", kp)
                    ser.write(b"l" if motor == "left" else b"r")
                    run_name = f"trial_{trial:02d}_{motor}_kp_{kp:g}"
                    data_filename = run_dir / (run_name + ".csv")
                    metadata, config, rows = collect_dataset(reader, motor, setpoint, kp, data_filename)
                    # MCU buffers released before configuring another run.
                    wait_for(reader, "Ready for next test")
                    save_plot(run_dir / (run_name + ".png"), motor, setpoint, kp, config, rows)
                    print(f"Saved {len(rows)} rows and plot to {data_filename}")
            finally:
                #Ask firmware to stop even after errors. Context closes the port.
                ser.write(b"e")

    #error handling for communication, waiting validation,and protocol handling
    except (SerialException, TimeoutError, ValueError, RuntimeError) as error:
        raise SystemExit(f"Collection failed: {error}") from error
    except KeyboardInterrupt: #allows for ctrl+c
        raise SystemExit("Collection interrupted; serial port closed.") #added


#copy the line folowing data to PC
def collect_line_dataset(reader, filename):
    #dealine for waiting
    deadline = monotonic() + FIRST_RESPONSE_TIMEOUT + LINE_DURATION_S
    line = reader.read(deadline)
    while not line.startswith("Begin data: "):
        line = reader.read(deadline)

    with filename.open("x", encoding="utf-8") as file:
        file.write("# " + line + "\n")
        try:
            while True:
                line = reader.read(monotonic() + BETWEEN_LINES_TIMEOUT)
                if line.startswith(END_MARKER):
                    file.write("# " + line + "\n")
                    if line != "ECompleted! that was the end of the data":
                        raise ValueError("Line run did not complete")
                    return
                file.write(line + "\n")
        except (Exception, KeyboardInterrupt):
            file.write("# Collection incomplete\n")
            raise


def plot_line_data(filename):
    from matplotlib import pyplot as plt

    times = []
    positions = []

    with filename.open() as file:
        reader = csv.DictReader(
            line for line in file if not line.startswith("#")
        )
        for row in reader:
            position = row["centroid_mm"].strip()
            if position.lower() in ("none", "nan", "undefined", ""):
                continue
            times.append(float(row["time_us"]) / 1_000_000)
            positions.append(float(position))

    plt.scatter(times, positions, s=10)
    plt.axhline(0, color="black", linestyle="--")
    plt.xlabel("elapsed time (s) from MCU")
    plt.ylabel("Centroid (mm)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename.with_suffix(".png"))
    plt.close()

#run, send settings for line following to romi, requests run saves data, attempts to stop
def run_line_mode():
    from serial import Serial

    if not LINE_FIRMWARE_READY:
        raise SystemExit("Matching board commands are not implemented yet.")

    OUTPUT_DIRECTORY.mkdir(exist_ok=True)
    run_dir = OUTPUT_DIRECTORY / datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")
    run_dir.mkdir()

    with Serial(
        SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=SERIAL_TIMEOUT,
        write_timeout=2.0,) as ser:
        reader = LineReader(ser)

        try:
            configure_number(
                ser, reader, b"v","Enter velocity setpoint [rad/s]:",float(LINE_BASE_SPEED),
            )
            configure_number(
                ser, reader, b"p","Enter outer [V/(rad/s)]:",float(LINE_KP),
            )

            ser.write(b"f")

            filename = run_dir / "line_following.csv"
            collect_line_dataset(reader, filename)
            print("Saved data to", filename)

        finally:
            ser.write(b"e")


if __name__ == "__main__":
    if RUN_MODE == "wheel":
        main()
    elif RUN_MODE == "line":
        run_line_mode()
    else:
        raise SystemExit("run in wheel or line mode")
