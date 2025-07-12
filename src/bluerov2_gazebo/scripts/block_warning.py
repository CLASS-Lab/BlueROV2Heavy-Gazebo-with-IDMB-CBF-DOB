#!/usr/bin/env python
import multiprocessing
import os
import pipes
import threading
import sys

import rospy

class suppress_TF_REPEATED_DATA(object):
    r'''Standard error filter used to suppress TF_REPEATED_DATA warning messages.
    '''
    def __init__(self):
        r'''Create a new warning suppressor object.
        '''
        self.__registered = False

    def __call__(self):
        r'''Replace the normal STDERR with a filter that suppresses
            `Warning: TF_REPEATED_DATA` messages.
        '''
        if self.__registered:
            return

        # Because the filter works by redirecting STDERR to a pipe, it's not possible to
        # then print error messages that are not meant to be suppressed. The solution is
        # to create a child process to print those messages in place of this process.
        queue = multiprocessing.Queue()
        def printer(queue):
            while True:
                line = queue.get()
                if line is None:
                    return

                sys.stderr.write(line)

        # Start the output process.
        self.__printer = multiprocessing.Process(target=printer, args=(queue,))
        self.__printer.start()

        # Preserve the normal STDERR.
        stderr_fileno = os.dup(2)
        stderr = sys.stderr

        # Create a pipe object and replace STDERR with it.
        piper = pipes.Template()
        pipe_name = rospy.get_name().replace('/', '_')
        pipe_out = piper.open(pipe_name, 'w')
        os.dup2(pipe_out.fileno(), 2)
        sys.stderr = pipe_out

        def read_pipe():
            skip = False

            # Open the pipe in read mode and read messages from it.
            with open(pipe_name) as pipe_in:
                while not rospy.is_shutdown():
                    line = pipe_in.readline()
                    if line.strip() == '':
                        continue

                    # The warning message is two lines long, so we need to also skip the line
                    # following the one starting with "Warning: TF_REPEATED_DATA".
                    if skip:
                        skip = False
                        continue

                    if line.startswith('Warning: TF_REPEATED_DATA'):
                        skip = True
                        continue

                    # If the line is not to be suppressed,
                    # send it to the child process for printing.
                    queue.put(line)

            # Stop the output process.
            queue.put(None)
            self.__printer.join()

            # Restore the normal STDERR.
            pipe_out.close()
            os.dup2(stderr_fileno, 2)
            sys.stderr = stderr

            self.__registered = False

        self.__reader = threading.Thread(target=read_pipe)
        self.__reader.start()

        self.__registered = True


# Turn the class into a singleton.
suppress_TF_REPEATED_DATA = suppress_TF_REPEATED_DATA()