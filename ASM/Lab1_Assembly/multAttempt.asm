
.text
li x20, 8			# val A
li x21, 4			# val B
li x23, 1
mv x24, x20			# accumulator

beq x21, x23, END

LOOP:
beq x21, x23, FINAL_ADD		# if x21 equal to 1
addi x21, x21, -2		# subtract 2
slli x24, x24, 1
beq x21, x0, FINAL_MULT
j LOOP 	

FINAL_ADD:
add x24, x24, x20
j END				# final add

FINAL_MULT:
slli x20, x20, 1
j END

END: