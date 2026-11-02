##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2012-2020 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2019 Zhiyuan Wan <dv.xw@qq.com>
## Copyright (C) 2019 Kongou Hikari <hikari@iloli.bid>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from common.srdhelper import SrdStrEnum

'''
OUTPUT_PYTHON format:

Packet:
[<ptype>, <pdata>]

<ptype>:
 - 'NEW STATE': <pdata> is the new state of the JTAG state machine.
   Valid values: 'TEST-LOGIC-RESET', 'RUN-TEST/IDLE', 'SELECT-DR-SCAN',
   'CAPTURE-DR', 'SHIFT-DR', 'EXIT1-DR', 'PAUSE-DR', 'EXIT2-DR', 'UPDATE-DR',
   'SELECT-IR-SCAN', 'CAPTURE-IR', 'SHIFT-IR', 'EXIT1-IR', 'PAUSE-IR',
   'EXIT2-IR', 'UPDATE-IR'.
 - 'IR TDI': Bitstring that was clocked into the IR register.
 - 'IR TDO': Bitstring that was clocked out of the IR register.
 - 'DR TDI': Bitstring that was clocked into the DR register.
 - 'DR TDO': Bitstring that was clocked out of the DR register.

All bitstrings are a list consisting of two items. The first is a sequence
of '1' and '0' characters (the right-most character is the LSB. Example:
'01110001', where 1 is the LSB). The second item is a list of ss/es values
for each bit that is in the bitstring.
'''

s = 'TEST-LOGIC-RESET RUN-TEST/IDLE \
     SELECT-DR-SCAN CAPTURE-DR UPDATE-DR PAUSE-DR SHIFT-DR EXIT1-DR EXIT2-DR \
     SELECT-IR-SCAN CAPTURE-IR UPDATE-IR PAUSE-IR SHIFT-IR EXIT1-IR EXIT2-IR'
St = SrdStrEnum.from_str('St', s)

jtag_states = [s.value for s in St]

s = 'EC SPARE TPDEL TPREV TPST RDYC DLYC SCNFMT CP OAC'.split()
s = ['CJTAG_' + x for x in s] + ['LEV2', 'CP1', 'CP2', 'CHK' ]
CtrlSt = SrdStrEnum.from_list('CtrlSt', s)
control_states = [s.value for s in CtrlSt]

s = ['OSCAN1', 'OSCAN2', 'FOUR_WIRE']
CSt = SrdStrEnum.from_list('CSt', s)

cjtag_states = [s.value for s in CSt]

cjtag_state_idx0 = len(jtag_states)
control_state_idx0 = cjtag_state_idx0 + len(cjtag_states)
tdi_idx = control_state_idx0 + len(control_states)
tdo_idx = tdi_idx + 1
tdi_bitstring_idx = tdi_idx + 2
tdo_bitstring_idx = tdi_idx + 3
tms_bits_idx = tdi_idx + 4

class Decoder(srd.Decoder):
    api_version = 3
    id = 'cjtag'
    name = 'cJTAG'
    longname = 'Compact Joint Test Action Group (IEEE 1149.7)'
    desc = 'Protocol for testing, debugging, and flashing ICs.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['jtag']
    tags = ['Debug/trace']
    channels = (
        {'id': 'tckc', 'name': 'TCKC', 'desc': 'Test clock'},
        {'id': 'tmsc', 'name': 'TMSC', 'desc': 'Test mode select'},
    )
    annotations = \
        tuple([tuple([s.lower(), s]) for s in jtag_states]) + \
        tuple([tuple([s.lower(), s]) for s in cjtag_states]) + \
        tuple([tuple([s.lower(), s]) for s in control_states]) + ( \
        ('bit-tdi', 'Bit (TDI)'),
        ('bit-tdo', 'Bit (TDO)'),
        ('bitstring-tdi', 'Bitstring (TDI)'),
        ('bitstring-tdo', 'Bitstring (TDO)'),
        ('bit-tms', 'Bit (TMS)'),
    )
    annotation_rows = (
        ('bits-tdi', 'Bits (TDI)', (tdi_idx,)),
        ('bits-tdo', 'Bits (TDO)', (tdo_idx,)),
        ('bitstrings-tdi', 'Bitstrings (TDI)', (tdi_bitstring_idx,)),
        ('bitstrings-tdo', 'Bitstrings (TDO)', (tdo_bitstring_idx,)),
        ('bits-tms', 'Bits (TMS)', (tms_bits_idx,)),
        ('control-states', 'control states',
            tuple(range(control_state_idx0, control_state_idx0 + len(control_states)))),
        ('cjtag-states', 'CJTAG states',
            tuple(range(cjtag_state_idx0, cjtag_state_idx0 + len(cjtag_states)))),
        ('jtag-states', 'JTAG states', tuple(range(len(jtag_states)))),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        # self.state = St.TEST_LOGIC_RESET
        self.state = St.RUN_TEST_IDLE
        self.cjtagstate = CSt.FOUR_WIRE
        self.oldcjtagstate = None
        self.ctrlstate = None
        self.oldctrlstate = None
        self.escape_edges = 0
        self.oaclen = 0
        self.oldtms = 0
        self.oacp = 0
        self.oscan1cycle = 0
        self.oldstate = None
        self.oldoldstate = None
        self.bits_tdi = []
        self.bits_tdo = []
        self.bits_samplenums_tdi = []
        self.bits_samplenums_tdo = []
        self.ss_item = self.es_item = None
        self.ss_bitstring = self.es_bitstring = None
        self.saved_item = None
        self.first = True
        self.first_bit = True
        self.zbs_count = 0
        self.control_level = 0
        self.cp = {}
        self.cp_num = 0
        self.chk_cnt = 0
        self.oscan_delay = 0

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        self.put(self.ss_item, self.es_item, self.out_ann, data)

    def putp(self, data):
        self.put(self.ss_item, self.es_item, self.out_python, data)

    def putx_bs(self, data):
        self.put(self.ss_bitstring, self.es_bitstring, self.out_ann, data)

    def putp_bs(self, data):
        self.put(self.ss_bitstring, self.es_bitstring, self.out_python, data)

    def advance_state_machine(self, tms):
        self.oldoldstate = self.oldstate
        self.oldstate = self.state

        if not self.ctrlstate is None and self.ctrlstate.value.startswith('CJTAG_'):
            self.oacp += 1
            if self.oacp > 4 and self.oaclen == 12:
                self.ctrlstate = CtrlSt.CJTAG_EC

            if self.oacp == 8 and tms == 0:
                self.oaclen = 36
            if self.oacp > 8 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_SPARE
            if self.oacp > 13 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_TPDEL
            if self.oacp > 16 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_TPREV
            if self.oacp > 18 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_TPST
            if self.oacp > 23 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_RDYC
            if self.oacp > 25 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_DLYC
            if self.oacp > 27 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_SCNFMT

            if self.oacp > 8 and self.oaclen == 12:
                self.ctrlstate = CtrlSt.CJTAG_CP
            if self.oacp > 32 and self.oaclen == 36:
                self.ctrlstate = CtrlSt.CJTAG_CP

            if self.oacp > self.oaclen:
                self.cjtagstate = CSt.OSCAN1
                self.oscan1cycle = 1
                self.state = St.TEST_LOGIC_RESET
            return

        # Intro "tree"
        if self.state == St.TEST_LOGIC_RESET:
            self.state = St.TEST_LOGIC_RESET if (tms) else St.RUN_TEST_IDLE
        elif self.state == St.RUN_TEST_IDLE:
            self.state = St.SELECT_DR_SCAN if (tms) else St.RUN_TEST_IDLE

        # DR "tree"
        elif self.state == St.SELECT_DR_SCAN:
            self.state = St.SELECT_IR_SCAN if (tms) else St.CAPTURE_DR
        elif self.state == St.CAPTURE_DR:
            self.state = St.EXIT1_DR if (tms) else St.SHIFT_DR
        elif self.state == St.SHIFT_DR:
            self.state = St.EXIT1_DR if (tms) else St.SHIFT_DR
        elif self.state == St.EXIT1_DR:
            self.state = St.UPDATE_DR if (tms) else St.PAUSE_DR
        elif self.state == St.PAUSE_DR:
            self.state = St.EXIT2_DR if (tms) else St.PAUSE_DR
        elif self.state == St.EXIT2_DR:
            self.state = St.UPDATE_DR if (tms) else St.SHIFT_DR
        elif self.state == St.UPDATE_DR:
            self.state = St.SELECT_DR_SCAN if (tms) else St.RUN_TEST_IDLE

        # IR "tree"
        elif self.state == St.SELECT_IR_SCAN:
            self.state = St.TEST_LOGIC_RESET if (tms) else St.CAPTURE_IR
        elif self.state == St.CAPTURE_IR:
            self.state = St.EXIT1_IR if (tms) else St.SHIFT_IR
        elif self.state == St.SHIFT_IR:
            self.state = St.EXIT1_IR if (tms) else St.SHIFT_IR
        elif self.state == St.EXIT1_IR:
            self.state = St.UPDATE_IR if (tms) else St.PAUSE_IR
        elif self.state == St.PAUSE_IR:
            self.state = St.EXIT2_IR if (tms) else St.PAUSE_IR
        elif self.state == St.EXIT2_IR:
            self.state = St.UPDATE_IR if (tms) else St.SHIFT_IR
        elif self.state == St.UPDATE_IR:
            self.state = St.SELECT_DR_SCAN if (tms) else St.RUN_TEST_IDLE

    def handle_rising_tckc_edge(self, tdi, tdo, tck, tms):

        # Rising TCK edges always advance the state machine.
        self.advance_state_machine(tms)

        if self.first:
            # Save the start sample and item for later (no output yet).
            self.ss_item = self.samplenum
            self.first = False
        else:
            # Output the saved item (from the last CLK edge to the current).
            self.es_item = self.samplenum
            # Output the old state (from last rising TCK edge to current one).
            self.putx([jtag_states.index(self.oldstate.value), [self.oldstate.value]])
            self.putp(['NEW STATE', self.state.value])

            self.putx([cjtag_state_idx0 + cjtag_states.index(self.oldcjtagstate.value),
                      [self.oldcjtagstate.value]])
            if not self.oldctrlstate is None:
                self.putx([control_state_idx0 + control_states.index(self.oldctrlstate.value),
                      [self.oldctrlstate.value]])
                if (self.oldctrlstate.value.startswith('CJTAG_')):
                    self.putx([tms_bits_idx, [str(self.oldtms)]])
        self.oldtms = tms

        # Upon SHIFT-*/EXIT1-* collect the current TDI/TDO values.
        if self.oldstate.value.startswith('SHIFT-') or \
                self.oldstate.value.startswith('EXIT1-'):
            if self.first_bit:
                self.ss_bitstring = self.samplenum
                self.first_bit = False
                if self.control_level == 2:
                    if self.oldstate.value == 'SHIFT-DR' or self.oldstate.value == 'EXIT1-DR':
                        self.cp_num += 1
                        if self.cp_num == 1:
                            self.ctrlstate = CtrlSt.CP1
                        elif self.cp_num == 2:
                            self.ctrlstate = CtrlSt.CP2

        if (not self.oldoldstate is None) and self.oldoldstate.value.startswith('SHIFT-'):
            self.putx([tdi_idx, [str(self.bits_tdi[0])]])
            self.putx([tdo_idx, [str(self.bits_tdo[0])]])
            # Use self.samplenum as ES of the previous bit.
            self.bits_samplenums_tdi[0][1] = self.samplenum
            self.bits_samplenums_tdo[0][1] = self.samplenum

        if self.oldstate.value.startswith('SHIFT-'):
            self.bits_tdi.insert(0, 'x' if tdi is None else tdi)
            self.bits_tdo.insert(0, 'x' if tdo is None else tdo)

            # Use self.samplenum as SS of the current bit.
            self.bits_samplenums_tdi.insert(0, [self.samplenum, -1])
            self.bits_samplenums_tdo.insert(0, [self.samplenum, -1])

        # Output all TDI/TDO bits if we just switched to UPDATE-*.
        if self.state.value.startswith('UPDATE-'):

            nbits = len(self.bits_tdi)
            if self.state.value == 'UPDATE-DR':
                if self.control_level == 0:
                    if nbits == 0:
                        self.zbs_count += 1
                    elif self.zbs_count > 0:
                        self.control_level = self.zbs_count
                        if self.control_level == 2:
                            self.ctrlstate = CtrlSt.LEV2
                        self.zbs_count = 0
                        self.cp_num = 0
                elif self.control_level == 2:
                    self.cp[self.cp_num] = nbits
                    self.ctrlstate = CtrlSt.LEV2

            self.es_bitstring = self.samplenum

            t = self.state.value[-2:] + ' TDI'
            b = ''.join(map(str, self.bits_tdi))
            h = '' if 'x' in b else ' (0x%x' % int('0b0' + b, 2) + ')'
            s = t + ': ' + b + h + ', ' + str(nbits) + ' bits'
            self.putx_bs([tdi_bitstring_idx, [s]])
            self.putp_bs([t, [b, self.bits_samplenums_tdi]])
            self.bits_tdi = []
            self.bits_samplenums_tdi = []

            t = self.state.value[-2:] + ' TDO'
            b = ''.join(map(str, self.bits_tdo))
            h = '' if 'x' in b else ' (0x%x' % int('0b0' + b, 2) + ')'
            s = t + ': ' + b + h + ', ' + str(len(self.bits_tdo)) + ' bits'
            self.putx_bs([tdo_bitstring_idx, [s]])
            self.putp_bs([t, [b, self.bits_samplenums_tdo]])
            self.bits_tdo = []
            self.bits_samplenums_tdo = []

            self.first_bit = True

            self.ss_bitstring = self.samplenum

        if self.oldstate.value == 'UPDATE-DR':
            if self.control_level == 2:
                if self.cp_num == 2:
                    self.cp_num = 0
                    if self.cp[1] == 3 and self.cp[2] >= 8:
                        self.oscan_delay = 2
                        self.chk_cnt = 3
                        self.ctrlstate = CtrlSt.CHK
                    if self.cjtagstate != CSt.FOUR_WIRE:
                        self.chk_cnt = 4
                        self.ctrlstate = CtrlSt.CHK

        if self.state.value == 'RUN-TEST/IDLE' and self.chk_cnt == 0:
            self.ctrlstate = CtrlSt.LEV2 if self.control_level == 2 else None

        if self.state.value == 'SELECT-IR':
            self.control_level = 0;
            self.ctrlstate = None

        if self.oscan_delay:
            self.oscan_delay-= 1
            if self.oscan_delay == 0:
                self.cjtagstate = CSt.OSCAN2 if self.cp[2] == 10 else CSt.OSCAN1
                self.oscan1cycle = 0
                self.state = St.TEST_LOGIC_RESET
                self.ctrlstate = CtrlSt.LEV2

        self.ss_item = self.samplenum

    def handle_tmsc_edge(self):
        self.escape_edges += 1

    def handle_tapc_state(self):
        self.oldcjtagstate = self.cjtagstate
        self.oldctrlstate = self.ctrlstate

        if self.escape_edges >= 8:
            self.cjtagstate = CSt.FOUR_WIRE
        if self.escape_edges == 6:
            self.ctrlstate = CtrlSt.CJTAG_OAC
            self.oacp = 0
            self.oaclen = 12

        self.escape_edges = 0

    def decode(self):
        tdi = tms = tdo = 0

        while True:
            # Wait for a rising edge on TCKC.
            tckc, tmsc = self.wait({0: 'r'})
            self.handle_tapc_state()

            if self.chk_cnt > 0:
                self.chk_cnt -= 1
            elif self.cjtagstate == CSt.OSCAN1 or (self.cjtagstate == CSt.OSCAN2 and self.state.value.startswith('SHIFT-')):
                if self.oscan1cycle == 0: # nTDI
                    tdi = 1 if (tmsc == 0) else 0
                    self.oscan1cycle = 1
                elif self.oscan1cycle == 1: # TMS
                    tms = tmsc
                    self.oscan1cycle = 2
                elif self.oscan1cycle == 2: # TDO
                    tdo = tmsc
                    self.handle_rising_tckc_edge(tdi, tdo, tckc, tms)
                    self.oscan1cycle = 0
            else:
                self.handle_rising_tckc_edge(None, None, tckc, tmsc)

            while (tckc == 1):
                tckc, tmsc_n = self.wait([{0: 'f'}, {1: 'e'}])
                if tmsc_n != tmsc:
                    tmsc = tmsc_n
                    self.handle_tmsc_edge()
