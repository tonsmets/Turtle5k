This is a directory that has scripts in it that can be runned on linux. A script can also be started when the computer starts. This is why this script is made.

for starting this script while booting the computer:
1-> sudo apt-get install ssh
2-> sudo apt-get install sshpass
3-> make the password of your computer = "Robocup". Or change this in the script
4-> put one of the files in /etc/init.d/....
5-> for both files: chmod +x /etc/init.d/"filename"
6-> make for both files a startup in the program : startupapplications
7-> example of a command = "gnome-terminal --execute /etc/init.d/ControlMotorsWithJoystickTwistStartupFile.sh"


