#!/usr/bin/env python

import sys
import argparse
import xml.etree.ElementTree as ET
from enum import Enum

# class that holds the configuration of the training/testing stages
class Config:
    # methods
    def __init__(self, xml_dir):
        self.__parser_status = True
        # retrive xml file-tree
        xml_tree = ET.parse(xml_dir)
        ## tag: 'input'
        xml_input = xml_tree.findall('input')
        self.__check_uniqueness('input', xml_input)
        # retrieve subtag: 'length'
        self.sequence_length = self.__retrieve_element('length', xml_input, int)
        # retrieve subtag: 'images'
        xml_images = xml_input[0].findall('images')
        self.__check_uniqueness('images', xml_images)
        self.im_format = self.__retrieve_element('format', xml_images, str).lower()
        self.im_width = self.__retrieve_element('width', xml_images, int)
        self.im_height = self.__retrieve_element('height', xml_images, int)
        # retrieve subtag: 'label'
        xml_label = xml_input[0].findall('label')
        self.__check_uniqueness('label', xml_label)
        self.start_label = self.__retrieve_element('start', xml_label, int)
        self.end_label = self.__retrieve_element('end', xml_label, int)
        # retrive multiple subtags: 'frame'
        xml_frames = xml_input[0].findall('frame')
        self.num_data_frames = len(xml_frames)
        self.frames = []
        for xml_frame in xml_frames:
            # retrive subtag: 'width'
            # NOTE frames are stored as tuples of the form: (type, width)
            self.frames.append( (xml_frame.attrib['type'], self.__retrieve_element('width', [xml_frame], int)) )

        ## tag: 'preprocessing'
        xml_preprocessing = xml_tree.findall('preprocessing')
        self.__check_uniqueness('preprocessing', xml_preprocessing)
        # retrieve subtag: 'filtering'
        self.filtering = self.__retrieve_element('filtering', xml_preprocessing, str).lower()

        ## tag: 'training'
        xml_training = xml_tree.findall('training')
        self.__check_uniqueness('training', xml_training)
        # retrieve subtag: 'path'
        self.dataset_path = self.__retrieve_element('path', xml_training, str)
        # retrieve subtag: 'hardware'
        self.training_hardware = self.__retrieve_element('hardware', xml_training, str).lower()
        # retrieve subtag: 'mode'
        self.training_mode = self.__retrieve_element('mode', xml_training, str).lower()

    def __retrieve_element(self, tag, xml_subtree, cast):
        # get list of all height tags on next level of xml_subtree
        xml_elements = xml_subtree[0].findall(tag)
        # check if only one tag is defined
        if self.__check_uniqueness(tag, xml_elements):
            return cast(xml_elements[0].text)
        else:
            return None

    def __check_uniqueness(self, tag, xml_subtree):
        ret = True
        if len(xml_subtree) == 0:
            print("Error: missing required tag '{0}'. It must be specified exactly once.".format(tag), file=sys.stderr)
            self.parser_success = ret = False
        elif len(xml_subtree) > 1:
            print("Error: ambigous definition of tag '{0}'. It must be specified exactly once.".format(tag), file=sys.stderr)
            self.parser_success = ret = False
        return ret

    def print_config(self):
        # TODO make nicer
        print("\nconfiguration overview:\n#######################")
        print(vars(self))

    def get_parser_status(self):
        return self.__parser_status


def main():
    # parse input arguments
    argparser = argparse.ArgumentParser(description="This stage takes in datasets and converts them according to the format specified in the passed config.xml file. The converted dataset will be used to train the model specified at the next stage.")
    argparser.add_argument('--config', '-conf', type=str, default=None, help="set path to config.xml file")
    args = argparser.parse_args()

    # load config file
    if args.config == None:
        print("Error: no config file given", file=sys.stderr)
        argparser.print_help()
        exit()
    conf = Config(args.config)

    # parse and check config file
    if not conf.get_parser_status():
        print("Error: something went wrong parsing the config.xml. Check if your config is specified properly.", file=sys.stderr)
        exit()
    else:
        print("'{0}' successfully parsed.".format(args.config))
        conf.print_config()

    # TODO go on here implementing functionality

if __name__ == "__main__":
    main()
