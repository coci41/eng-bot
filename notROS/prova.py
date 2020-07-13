import RPi.GPIO as io
import sys, tty, termios, time
import gpiozero


# The getch method can determine which key has been pressed
# by the user on the keyboard by accessing the system files
# It will then return the pressed key as a variable
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


robot = gpiozero.Robot(left=(24,23,25), right=(22,27,17))
forward_speed = 0.35
turn_speed = 0.3
# Infinite loop that will not end until the user presses the
# exit key
while True:
    # Keyboard character retrieval method is called and saved
    # into variable
    char = getch()

    # The car will drive forward when the "w" key is pressed
    if(char == "w"):
        robot.forward(forward_speed)

    # The car will reverse when the "s" key is pressed
    #if(char == "s"):
        #motor2_reverse()
        #motor2.ChangeDutyCycle(99)

    # The "a" key will toggle the steering left
    if(char == "a"):
        robot.left(turn_speed)

    # The "d" key will toggle the steering right
    if(char == "d"):
        robot.right(turn_speed)

    # The "x" key will break the loop and exit the program
    if(char == "x"):
        print("Program Ended")
        break

    # At the end of each loop the acceleration motor will stop
    # and wait for its next command
    #motor2.ChangeDutyCycle(0)

    # The keyboard character variable will be set to blank, ready
    # to save the next key that is pressed
    char = ""

# Program will cease all GPIO activity before terminating
io.cleanup()