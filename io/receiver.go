package io

import (
	"fmt"
	"net"
)

type CmdData struct {
	Yaw      int16
	Pitch    int16
	Roll     int16
	Throttle int16
	Aux1     int16
	Aux2     int16
}

/**
* Listen/Accept a connection, then process incoming commands
**/
func ReadCommands(cmdChannel chan CmdData) {
	for {
		ln, err := net.Listen("tcp", ":8042")
		if err != nil {
			fmt.Printf("ReadCommands: Listen() failed, err=%v\n", err)
			close(cmdChannel)
			return
		}
		for {
			fmt.Printf("ReadCommands: listening for a connection on 8042\n")
			conn, err := ln.Accept()
			if err != nil {
				fmt.Printf("ReadCommands: Accept() failed, err=%v\n", err)
				close(cmdChannel)
				return
			}
			fmt.Printf("ReadCommands: accepted a connection from %s\n", conn.RemoteAddr().String())
			readCmds(cmdChannel, conn)
		}
	}
}

/**
* Read incoming commands through a socket into a CmdData and send it through cmdChannel.
* Expecting 12 bytes, 6 pairs of 16 bit signed integers in network byte order.
* (yaw, pitch, roll, throttle, aux1, aux2)
**/
func readCmds(cmdChannel chan CmdData, conn net.Conn) {
	var (
		data CmdData
		buf  [12]byte
		n    int
		err  error
	)
	defer conn.Close()
	for {
		n, err = conn.Read(buf[n:])
		if err != nil {
			// err == os.EOF
			fmt.Printf("readCmds: Read() failed, err=%v\n", err)
			continue
		} else if n < 12 {
			continue
		}
		data.Yaw      = int16(uint16(buf[0])<<8  | uint16(buf[1]))
		data.Pitch    = int16(uint16(buf[2])<<8  | uint16(buf[3]))
		data.Roll     = int16(uint16(buf[4])<<8  | uint16(buf[5]))
		data.Throttle = int16(uint16(buf[6])<<8  | uint16(buf[7]))
		data.Aux1     = int16(uint16(buf[8])<<8  | uint16(buf[9]))
		data.Aux1     = int16(uint16(buf[10])<<8 | uint16(buf[11]))
		cmdChannel <- data
		n = 0
	}
}
