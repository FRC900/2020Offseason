"""
Compares two paths (which can be selected via command line flags "control" and "test")
"""
import sys
import rospy

def get_path_control():


def get_path_test():


def usage:
    return "run with 1 argument (control or test)"

if __name__ == "__main__":
    if len(sys.argv) == 2:
        path == string(sys.argv[1])
    else:
        print(usage)
        sys.exit(1)
    
    if path == "control"
        print(get_path_control())
    elif path == "test":
        print(get_path_test())
    else 
        print(usage)
        sys.exit(1)