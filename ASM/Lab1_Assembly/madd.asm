.data 
MatrixA: .word 7, 2, 8, 6, 5, 8
MatrixB: .word 3, 4, 5, 6, 2, 4
MatrixC: .word

.equ MatrixSize, 2	# matrix will be equal to: MatrixSize * MatrixSize

.text
la x28, MatrixA		# store pointers to each matrix array
la x29, MatrixB
la x30, MatrixC
li x31, 0		# current operation number
li x12, 6		# total number of spaces stored here

LOOP:
lw x10, 0(x28)		# load val of MatrixA
lw x11, 0(x29)		# load val of MatrixB
add x11, x10, x11	# add vals
sw x11, 0(x30)		# store val in MatrixC
addi x31, x31, 1	# increment fill counter

beq x31, x12, END 	# check if all spaces in MatrixC are now filled

addi x28, x28, 4	# increment pointers 
addi x29, x29, 4
addi x30, x30, 4

j LOOP


END:
