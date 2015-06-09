#!/usr/bin/env python


class MyClass:
    def __init__(self):
        self.myvar = 1
        self.myvar2 = "hola"

    def do_stuff(self):
        while True:
            print self.myvar


if __name__ == '__main__':
    m = MyClass()
    m.do_stuff()
