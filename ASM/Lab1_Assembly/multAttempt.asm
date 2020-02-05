
.text
li x20, 6			# val A
li x21, 0			# val B
mv x22, x0			# accumulator

LOOP:
beq x21, x0, END
add x22, x22, x20
addi x21, x21, -1
j LOOP
 
END:
