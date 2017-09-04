"""Files tests simple file read related operations"""

class SimpleFile(object):
    """SimpleFile tests using file read api to do some simple math"""
    def __init__(self, file_path):
        self.numbers = []

        with open(file_path) as f:
            content = f.read().splitlines()

        self.numbers = [x.split(" ") for x in content]



    def get_mean(self, line_number):
        """
        get_mean retrieves the mean value of the list by line_number (starts
        with zero)
        """
        x = list(map(int, self.numbers[line_number]))
        return sum(x)/len(x)

    def get_max(self, line_number):
        """
        get_max retrieves the maximum value of the list by line_number (starts
        with zero)
        """
        x = list(map(int, self.numbers[line_number]))
        return max(x)

    def get_min(self, line_number):
        """
        get_min retrieves the minimum value of the list by line_number (starts
        with zero)
        """
        x = list(map(int, self.numbers[line_number]))
        return min(x)

    def get_sum(self, line_number):
        """
        get_sum retrieves the sumation of the list by line_number (starts with
        zero)
        """
        x = list(map(int, self.numbers[line_number]))
        return sum(x)
