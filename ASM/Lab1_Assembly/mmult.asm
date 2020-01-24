
.data
MatrixA: .word 1, 2, 3, 4, 5, 6, 7, 8, 9    # 3x3 matrix	    A:	1 2 3   B: 5 8 2  C: 
MatrixB: .word 5, 6, 7, 8, 9, 1, 2, 3, 4    # 3x3 matrix		4 5 6	   6 9 3
MatrixC: .word				    # 				7 8 9	   7 1 4
	
.text
la x9, MatrixA		# store addresses of the matricies
la x18, MatrixB
la x19, MatrixC
mv x20, x0		# accumulator
mv x21, x0		# row tracker
mv x22, x0		# col tracker
li x23, 3		# size of matrix - CHANGE BASED ON SIZE
mv x24, x0		# tracker to determine end

GET_THREE:
BEQ x21, x23, NEXT_COL
lw x25, 0(x9)		# load Val A
lw x26, 0(x18)		# load Val B
call MULT		# do the multiplication
add x20, x20, x27	# add result to accumulator
addi x9, x9, 4		# get next two addresses
addi x18, x18, 4
addi x21, x21, 1	# track current column
j GET_THREE


NEXT_COL:
sw x20, 0(x19)
addi x22, x22, 1	# increment col
addi x19, x19, 4	# increment MatrixC address
mv x20, x0		# reset accumulator
mv x21, x0		
BEQ x22, x23, NEXT_ROW
addi x9, x9, -12		# reset MatrixA address for next col - CHANGE BASED ON SIZE
j GET_THREE

NEXT_ROW:
addi x24, x24, 1
BEQ x24, x23, DONE
addi x18, x18, -36	# reset MatrixB address - CHANGE BASED ON SIZE
mv x22, x0		# reset col
j GET_THREE

MULT:
mv x27, x0			# accumulator
LOOP:
beq x26, x0, END
add x27, x27, x25
addi x26, x26, -1
j LOOP
END:
ret 

DONE:			# end is reached
