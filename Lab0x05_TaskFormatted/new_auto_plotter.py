
"""Collect one serial dataset, save it as CSV, and create a plot.
This is starter code rather than a complete Lab 0x04 solution. It shows a
robust pattern for one command and one dataset. Students will need to adapt the
command sequence, expected messages, and test loop to match their firmware.
"""
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
SERIAL_PORT = "COM5"
BAUDRATE = 115_200
SERIAL_TIMEOUT = 0.25

# Single-character commands are sent as soon as they are chosen; they do not
# need a line ending. Students will extend this into the duty-cycle and
# two-motor command sequence. A multicharacter numeric value does need the line
# ending expected by the firmware so that it knows when entry is complete.
#** One carriage return completes entry without leaving a second terminator queued.
NUMERIC_LINE_ENDING = "\r" #changed

#Here we test duty cycle by reeplacing the starter's single START_COMMAND with two motor/duty pairs
TESTS = (("left", 25.0), ("right", -25.0)) #added
# Stop waiting and report an error instead of hanging forever if the firmware
# does not respond or stops transmitting partway through a dataset.
# monotonic() returns a steadily increasing time in seconds. Its starting value
# is arbitrary, so it is not used as a date or time of day. Subtracting an old
# reading from a new reading gives the elapsed time without being affected if
# the computer's clock is adjusted while the program is running.
#** FIRST_RESPONSE_TIMEOUT also covers command acknowledgements and the beginning marker.
FIRST_RESPONSE_TIMEOUT = 30.0
BETWEEN_LINES_TIMEOUT = 5.0
# The firmware should print this marker on its own line after the last data row.
# Using an explicit marker is more reliable than assuming that a quiet serial
# port means the dataset is complete.
END_MARKER = "End of data"

#Here match the firware headers
HEADERS = ["time_us", "position_counts"] #added
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
    #** Identify which firmware response is missing when a timeout occurs.
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
            return #added
        #check for firmwareerrors
        if "out of range" in line or line == "Motor test busy": #added
            raise RuntimeError(line) #added

#waits for the motor and duty cycle begining markers, collects valid rows fo nums, returns rows
def collect_dataset(reader, motor, duty): #changed
    # Check motor and duty with a beginning marker to identify the requested
    #response before interpreting following lines as numeric data.
    marker = f"Begin data: motor={motor},duty_cycle_pct={duty}" #added
    wait_for(reader, marker) #added
    wait_for(reader, ",".join(HEADERS)) #added-join headers with commas
    rows = [] #changed - empty list for samples
    deadline = monotonic() + BETWEEN_LINES_TIMEOUT #time for next compelte line
    while True: #recieve lines ontil complete or exception stops the collecting
        #one complete doceded line
        line = reader.read(deadline)
        if line == END_MARKER: #does the line equal the end of data
            if not rows: #is dataset empty
                raise ValueError("Dataset contains no numeric rows") #changed
            return rows #end and return samples collected
        content = line.split("#", 1)[0].strip()#remove inline comment and the whitespace around
        fields = [field.strip() for field in content.split(",")] #seperate field strings for numeric values
        try: #had helpf from copilot to format these value errors  for NaN
            if len(fields) != len(HEADERS): #row with diff num of fields than expected
                raise ValueError("wrong number of columns") 
            values = [float(field) for field in fields] #convert text fields to floats
            if not all(math.isfinite(value) for value in values): #detect NaN or infinite values
                raise ValueError("nonfinite data") #added
        except ValueError as error: #changed
           #Handle field count and nomber fails together
            if "," in content: #added
                raise ValueError(f"Malformed data row: {line}") from error 
            #shwo the rejected data
            print(f"Serial message: {line}")
            continue
        rows.append(values) #add the values in together
        #update the deadline
        deadline = monotonic() + BETWEEN_LINES_TIMEOUT #changed

#write the collected measurements into csv
def save_csv(filename, motor, duty, rows): #changed
    #identifies the motor and signed duty no matter the file
    # csv.writer does CSV formatting 
    with filename.open("x", encoding="utf-8", newline="") as csv_file: #changed
        csv_file.write(f"# motor={motor}, duty_cycle_pct={duty}\n") #added
        writer = csv.writer(csv_file) #added
        writer.writerow(HEADERS) #added
        writer.writerows(rows) #added

#save the measurements as plots with matplotlib
def save_plot(filename, motor, duty, rows): #changed
    from matplotlib import pyplot
    figure, axes = pyplot.subplots()
    #make the firmware's microseconds as seconds for plots
    #plot encoder counts
    # label motor type (left of right) and signed duty cycle.
    axes.plot([row[0] / 1_000_000 for row in rows], #changed
              [row[1] for row in rows], label="Encoder position") #changed
    axes.set_xlabel("Time [s]") #x-axis-time in seconds
    axes.set_ylabel("Encoder position [counts]") # y axis encoder position
    axes.set_title(f"{motor.title()} motor: {duty:+g}% duty cycle") #changed
    axes.grid(True)
    axes.legend()
    figure.tight_layout()
    #save plot without overwriting current file
    try:
        with filename.open("xb") as output: #"xb" create new file in binary mode, 
            #with to close file when block ends
            #select the png format explicitly
            figure.savefig(output, format="png", dpi=200) #changed
    finally: #closes the figure
        pyplot.close(figure)

# Calculate interval velocities from the existing time/position dataset.
def calculate_velocity(rows):
    """Return midpoint times [s] and signed interval velocities [counts/s]."""
    # Require two position measurements to calculate a change in position.
    if len(rows) < 2:
        # Report insufficient data rather than save an empty velocity graph.
        raise ValueError("Velocity calculation requires at least two samples")
    # Create lists to hold numeric plot coordinates on the PC.
    times = []
    velocities = []
    # Pair each position/time row with the next row in the dataset.
    for previous, current in zip(rows, rows[1:]):
        # Use actual recorded elapsed time, converting microseconds to seconds.
        dt = (current[0] - previous[0]) / 1_000_000
        # Reject duplicate or backwards timestamps that invalidate the derivative.
        if not math.isfinite(dt) or dt <= 0:
            # Identify a dataset timing problem rather than divide by zero.
            raise ValueError("Velocity calculation requires increasing finite timestamps")
        # Calculate signed displacement divided by the actual measurement interval.
        velocity = (current[1] - previous[1]) / dt
        # Check the numeric result before passing it to Matplotlib.
        if not math.isfinite(velocity):
            # Report invalid position data or a nonfinite calculated velocity.
            raise ValueError("Calculated encoder velocity must be finite")
        # Place each interval-average velocity at that interval's midpoint in seconds.
        times.append((previous[0] + current[0]) / (2 * 1_000_000))
        # Store the numeric velocity value; retain the encoder's original sign.
        velocities.append(velocity)
    # Return the two coordinate lists to the plotting function.
    return times, velocities


# Save a separate velocity plot for whichever motor dataset is supplied.
def save_velocity_plot(filename, motor, duty, rows):
    """Plot interval-average encoder velocity against actual elapsed time.

    Uses counts/s until verified counts-per-wheel-revolution and side signs
    are available. This is derived from logged positions, not the controller's
    internal velocity measurement. N positions produce N-1 velocity points.
    """
    # Import the plotting module without opening or using a serial connection.
    from matplotlib import pyplot
    # Call the numeric helper with this encoder's collected data as its argument.
    times, velocities = calculate_velocity(rows)
    # Construct a Figure object and obtain its Axes object for drawing the plot.
    figure, axes = pyplot.subplots()
    # Ensure the figure closes even if plotting or saving fails.
    try:
        # Call the Axes.plot() method with time as x and velocity as y.
        axes.plot(times, velocities, marker=".", label=f"{motor.title()} encoder velocity")
        # Label the recorded test time in seconds.
        axes.set_xlabel("Time [s]")
        # Label the units that can be calculated without assuming encoder scale.
        axes.set_ylabel("Encoder velocity [counts/s]")
        # Identify the encoder/motor side and the signed test duty.
        axes.set_title(f"{motor.title()} motor: {duty:+g}% duty cycle — velocity")
        # Add grid lines to make sample spacing and velocity increments visible.
        axes.grid(True)
        # Display the plotted encoder's label.
        axes.legend()
        # Fit the plot labels within the figure boundary.
        figure.tight_layout()
        # Create a new output file without overwriting an existing plot.
        with filename.open("xb") as output:
            # Save the Figure object as a PNG image.
            figure.savefig(output, format="png", dpi=200)
    finally:
        # Close this Figure object to release its plotting resources.
        pyplot.close(figure)


#uses the serial port to run the left and right tests together
def main():
    #import pyserial class and serial exception class
    from serial import Serial, SerialException
    #check if motor is in the tests and error if not configured right
    if [motor for motor, _ in TESTS] != ["left", "right"]: #added
        raise ValueError("Configure one left test followed by one right test") #added
    for _, duty in TESTS: #added-make sure the duty cycles are within range
        if not math.isfinite(duty) or not -100 <= duty <= 100:
            raise ValueError("Configured duty cycles must be within -100 to +100")
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
            #Process configured tests one by one
            for motor, configured_duty in TESTS: #added- help from copilot to get formating for cofigure 
                #duty as float then back to text
                duty = float(configured_duty) #convert duty to float
                ser.write(b"d") #added-send without newline
                wait_for(reader, "Enter duty cycle [%]:") #added-wait for firmware
                ser.write((str(duty) + NUMERIC_LINE_ENDING).encode("ascii")) #changed
                wait_for(reader, f"Value set to {duty}") #added
                ser.write(b"l" if motor == "left" else b"r") #convert duty to text
                #wait for the matching dataset, colelcts measurements, returns when marker arrives
                rows = collect_dataset(reader, motor, duty) 
                RUN_NAME = f"{motor}_{duty:+g}pct" #displays the sign
                data_filename = run_dir / f"{RUN_NAME}.csv" #save as csv in run dir
                plot_filename = run_dir / f"{RUN_NAME}.png" #save as png in run dir
                save_csv(data_filename, motor, duty, rows) #save csv
                save_plot(plot_filename, motor, duty, rows) #gen plots
                # Store a Path object for this motor's additional velocity image.
                velocity_filename = run_dir / f"{RUN_NAME}_velocity.png"
                # Pass the same collected rows to the new velocity plotting function.
                save_velocity_plot(velocity_filename, motor, duty, rows)
                print(f"Saved {len(rows)} rows and plot for {motor} to {run_dir}") #report sample and output loc

    #error handling for communication, waiting validation,and protocol handling
    except (SerialException, TimeoutError, ValueError, RuntimeError) as error:
        raise SystemExit(f"Collection failed: {error}") from error
    except KeyboardInterrupt: #allows for ctrl+c
        raise SystemExit("Collection interrupted; serial port closed.") #added


if __name__ == "__main__":
    main()
