# -*- coding: utf-8 -*-
"""
Created on Tue Nov  1 13:17:17 2016

@author: mapper
"""

def convert2float(val):
    """ Converts the input to a float if possible. If it is not it returns the 
        input
    Input:
        val - Value to be converted to an float
    Output:
        new_value - Value that was converted to an float if possible
    """
    try:
        new_val= float(val)
    except ValueError:
        new_val = val
    return new_val

def parse_file(filename, invalid_char = '//', delimiter1 ='=', delimiter2 =','):
    """ This function reads in a file and makes a list of list of the parsed 
        information on the files. This function will skip any lines that are 
        blank or have the invalid character defined in the input.
        
    Inputs:
        filename - Name of the file to be parsed
        invalid_char - Character that determines that line is invalid
        delimiter1 - Character used to specify the first boundary between
                independent regions in the lines in the file
        delimiter2 - Character used to specify the second boundary between
                independent regions in the lines in the file
            EX: ENCs = US5NH01, US5NH02
                    where '=' is delimiter1 and ',' is delimiter2
    
    Output:
        list_of_lists - A list containing a lists of the parsed information in 
                    the file
    """
    list_of_lists = []
    with open(filename) as f:
        for line in f:
            # If the line does not contain the invalid character or is not an  
            #   empty space, parse the line
            if (not line.isspace() and invalid_char not in line):
                inner_list = [elt.strip() for elt in line.split(delimiter1)]
                # If there is a list of 
                if (delimiter2 in inner_list[1]):
                    inner_list2 = [elt2.strip() for elt2 in inner_list[1].split(delimiter2)]
                    for i in range(len(inner_list2)):
                        inner_list2[i]=convert2float(inner_list2[i])
                    inner_list[1] = inner_list2
                else:
                    # Convert to a float if it is a number
                    inner_list[1]=convert2float(inner_list[1])
                list_of_lists.append(inner_list)
    return list_of_lists