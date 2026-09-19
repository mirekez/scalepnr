// db2fasm emits LUT INIT bits, but has no separate INV configuration emitter.
module INV(input I, output O);
    LUT1 #(.INIT(2'b01)) _TECHMAP_REPLACE_ (.I0(I), .O(O));
endmodule
