recFib :: Num a => a -> a -> [a]
recFib a b = (a + b) : recFib b (a+b)

fibonacci :: Num a => [a]
fibonacci = [0, 1] ++ recFib 0 1