classdef LevenbergMarquardtStatus < int16
    enumeration
        NOT_INTIALIZED  (-2)
        ITERATION_LIMIT (-1)
        SOLVING         (0)
        LOCAL_MINIMUM   (1)
        X_TOLERANCE     (2)
        FUN_TOLERANCE   (3)
    end
end

