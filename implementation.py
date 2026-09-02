"""
NanoSense Core Architecture Simulator
Models Booth's Multiplication, Restoring Division, IEEE-754 Addition, 
Pipeline Hazards, and Cache Access for ultra-low-power IoT nodes.
"""

# ==========================================
# TASK 1: Arithmetic Logic Unit (ALU)
# ==========================================

def arithmetic_shift_right(val, bits):
    """Helper for Booth's Algorithm"""
    sign_bit = val & (1 << (bits - 1))
    return (val >> 1) | sign_bit

def booths_multiplier_8bit(multiplicand, multiplier):
    """Simulates an 8-bit signed Booth's Multiplier"""
    print(f"--- Booth's Multiplication: {multiplicand} * {multiplier} ---")
    
    # Convert to 8-bit two's complement integers
    M = multiplicand & 0xFF
    Q = multiplier & 0xFF
    A = 0x00
    Q_minus_1 = 0
    
    for cycle in range(8):
        Q_0 = Q & 1
        # Check Booth's conditions (Q_0, Q_minus_1)
        if Q_0 == 1 and Q_minus_1 == 0:
            A = (A - M) & 0xFF  # Subtract M
        elif Q_0 == 0 and Q_minus_1 == 1:
            A = (A + M) & 0xFF  # Add M
            
        # Arithmetic Shift Right [A, Q, Q_minus_1]
        combined = (A << 9) | (Q << 1) | Q_minus_1
        sign_bit = combined & (1 << 17)
        combined = (combined >> 1) | sign_bit
        
        A = (combined >> 9) & 0xFF
        Q = (combined >> 1) & 0xFF
        Q_minus_1 = combined & 1

    result = (A << 8) | Q
    # Convert 16-bit 2's complement to signed integer
    if result & (1 << 15):
        result -= (1 << 16)
        
    print(f"Result: {result} (Cycles: 8)\n")
    return result

def restoring_divider_8bit(dividend, divisor):
    """Simulates an 8-bit unsigned Restoring Divider"""
    print(f"--- Restoring Division: {dividend} / {divisor} ---")
    if divisor == 0:
        raise ZeroDivisionError("Hardware fault: Divide by zero")
        
    A = 0
    Q = dividend & 0xFF
    M = divisor & 0xFF
    
    for cycle in range(8):
        # Shift Left [A, Q]
        A = ((A << 1) | ((Q >> 7) & 1)) & 0xFF
        Q = (Q << 1) & 0xFF
        
        # Subtract Divisor
        A = (A - M) & 0xFF
        
        if A & 0x80: # If A is negative (MSB is 1)
            Q = Q & ~1 # Set Q[0] = 0
            A = (A + M) & 0xFF # Restore A
        else:
            Q = Q | 1 # Set Q[0] = 1
            
    print(f"Quotient: {Q}, Remainder: {A} (Cycles: 8)\n")
    return Q, A


# ==========================================
# TASK 2: IEEE-754 Floating Point Addition
# ==========================================

import struct

def float_to_bin(f):
    """Helper to get 32-bit integer representation of float"""
    return struct.unpack('>I', struct.pack('>f', f))[0]

def ieee754_add(f1, f2):
    """Models hardware alignment and normalization for float addition"""
    print(f"--- IEEE-754 Float Addition: {f1} + {f2} ---")
    b1, b2 = float_to_bin(f1), float_to_bin(f2)
    
    # Extract fields (assuming positive numbers for simplicity in this model)
    exp1 = (b1 >> 23) & 0xFF
    exp2 = (b2 >> 23) & 0xFF
    mant1 = (b1 & 0x7FFFFF) | 0x800000 # Add hidden bit
    mant2 = (b2 & 0x7FFFFF) | 0x800000 
    
    # Align Exponents
    if exp1 > exp2:
        shift = exp1 - exp2
        mant2 >>= shift
        final_exp = exp1
    else:
        shift = exp2 - exp1
        mant1 >>= shift
        final_exp = exp2
        
    # Add Mantissas
    result_mant = mant1 + mant2
    
    # Normalize
    if result_mant & 0x1000000: # Overflow in mantissa (carried out of 24th bit)
        result_mant >>= 1
        final_exp += 1
        
    # Check for Exponent Overflow
    if final_exp >= 255:
        print("Hardware Exception: Float Overflow! Clamping to Infinity.\n")
        return float('inf')
        
    result_mant &= 0x7FFFFF # Remove hidden bit
    result_bin = (final_exp << 23) | result_mant
    
    result_float = struct.unpack('>f', struct.pack('>I', result_bin))[0]
    print(f"Hardware Result: {result_float}\n")
    return result_float


# ==========================================
# TASK 3: Pipeline Hazard Simulation
# ==========================================

def evaluate_pipeline_hazards():
    """Models cycle counts for sensor fusion sequence with and without forwarding"""
    print("--- 5-Stage Pipeline Simulation ---")
    print("Sequence: LW R1 -> ADD R3, R1 -> SW R3")
    
    base_cycles = 5 # Time for first instruction to traverse IF-ID-EX-MEM-WB
    num_instructions = 3
    
    # 1. Without Forwarding: Data hazard on R1 requires 2 stall cycles.
    # LW finishes MEM at cycle 4, writes at 5. ADD needs R1 at ID (cycle 3).
    stalls_without = 2 
    total_cycles_no_fwd = (base_cycles - 1) + num_instructions + stalls_without
    
    # 2. With EX/MEM Forwarding: 
    # LW data ready after MEM (cycle 4). ADD needs it at EX (cycle 4). 1 load-use stall.
    stalls_with = 1
    total_cycles_fwd = (base_cycles - 1) + num_instructions + stalls_with
    
    print(f"Cycles without forwarding: {total_cycles_no_fwd}")
    print(f"Cycles with EX/MEM forwarding: {total_cycles_fwd}")
    print(f"Power Savings: {(total_cycles_no_fwd - total_cycles_fwd) / total_cycles_no_fwd * 100:.1f}%\n")


# ==========================================
# TASK 4: Cache vs. Scratchpad (TCM)
# ==========================================

class DirectMappedCache:
    """Simulates a direct-mapped cache for the circular sensor buffer"""
    def __init__(self, size_bytes=16, block_bytes=4):
        self.num_blocks = size_bytes // block_bytes
        self.block_size = block_bytes
        self.cache_tags = [None] * self.num_blocks
        self.hits = 0
        self.misses = 0
        
    def access(self, address):
        block_address = address // self.block_size
        index = block_address % self.num_blocks
        tag = block_address // self.num_blocks
        
        if self.cache_tags[index] == tag:
            self.hits += 1
        else:
            self.misses += 1
            self.cache_tags[index] = tag # Fetch block into cache

def evaluate_memory_strategy():
    print("--- Memory Strategy Simulation ---")
    # Simulating fetching 32-bit (4-byte) floats sequentially from a circular buffer
    access_trace = [0, 4, 8, 12, 16, 20, 24, 28] 
    
    # 1. Using Standard Data Cache
    d_cache = DirectMappedCache(size_bytes=16, block_bytes=4)
    for addr in access_trace:
        d_cache.access(addr)
        
    total_accesses = len(access_trace)
    miss_rate = (d_cache.misses / total_accesses) * 100
    
    print(f"Standard D-Cache Miss Rate: {miss_rate}% (Compulsory misses on streaming data)")
    print("Conclusion: D-Cache tag comparisons waste power. Use Scratchpad/TCM for 1-cycle guaranteed access.\n")

# ==========================================
# RUN SIMULATIONS
# ==========================================
if __name__ == "__main__":
    booths_multiplier_8bit(5, -3)
    restoring_divider_8bit(14, 3)
    ieee754_add(1.5, 2.75)
    
    # Trigger Edge Case: Overflow
    ieee754_add(3.4e38, 3.4e38) 
    
    evaluate_pipeline_hazards()
    evaluate_memory_strategy()