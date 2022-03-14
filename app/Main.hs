{-# LANGUAGE TupleSections, FlexibleContexts, TemplateHaskell, Rank2Types, NoMonomorphismRestriction #-}
{-# LANGUAGE DeriveGeneric #-}

module Main where

import Control.Lens
import Linear (Additive, M22(..), V2(..), (!*), (*^), (^+^), (^-^), (!-!), (!*!), norm, crossZ)
import Data.Map as M (insert, lookup, Map, empty, toList, fromList)
import Data.List (transpose)
import System.Exit (die)
import Control.Monad (forM)
import Graphics.Gloss as G
import Graphics.Gloss.Interface.IO.Game
import Debug.Trace (traceShow)
import Text.Printf

---------------------------
-- Util
---------------------------

(*.) :: V2 Float -> V2 Float -> Float
(V2 x0 y0) *. (V2 x1 y1) = x0 * y1 - y0 * x1


(&$) :: (Show b) => a -> b -> a
a &$ b = traceShow b a

infixl 1 &$

rotateVec :: Float -> V2 Float -> V2 Float
rotateVec t (V2 x y) = V2 (c * x - s * y) (s * x + c * y)
    where
    c = cos t
    s = sin t

assert :: (Show s) => Bool -> s -> a -> a
assert b s = if b then id else error (show s)

----------------------------
-- regular matrix utility
----------------------------

shape :: [[Float]] -> (Int, Int)
shape m@(r:rs) = (length(r), length(m))


zerolike :: [Float] -> [Float]
zerolike v = replicate (length v) 0


ithVec :: Float -> Int -> Int -> [Float]
ithVec x 0 n = x:(replicate n 0)
ithVec x i n = 0:(ithVec x (i-1) (n-1))


diagMat :: [[Float]] -> [[Float]]
diagMat mat = [ithVec ((mat !! i) !! i) i n | i <- [0..(n-1)]]
    where
    n = length mat


rdiagMat :: [[Float]] -> [[Float]]
rdiagMat mat = [ithVec (per ((mat !! i) !! i)) i n | i <- [0..(n-1)]]
    where
    n = length mat
    per 0 = 0
    per f = 1/f


gaussSeidel1 :: [[Float]] -> [Float] -> [Float] -> [Float]
gaussSeidel1 matA b x0 = (rdiagMat matA) !* (b ^-^ (matA !-! diagMat matA) !* x0)


-- should be clamped each step
gaussSeidel :: [[Float]] -> [Float] -> Int -> [Float]
gaussSeidel matA b n = (iterate (gaussSeidel1 matA b) (zerolike b)) !! n


---------------------------
-- Lib
---------------------------


type Position = V2 Float
type Attitude = Float     -- [rad] not Quaternion
type Mass = Float
type Inertia  = Float -- not M22 Float
type TimeDelta = Float
type Time = Float
type Force = V2 Float
type Torque = Float
type Mat = [[Float]]
type Vec = [Float]
type PushFactor = Float

deg2rad = (*) (pi / 180)

data RigidBody = RigidBody
    { _rmass    :: Mass   -- mass^-1
    , _rinertia :: Inertia -- inertia^-1
    , _cog      :: Position
    , _att      :: Attitude
    , _lmoment  :: V2 Float
    , _amoment  :: Float
    } deriving Show

makeLenses ''RigidBody


--updateMomentWithForce  :: TimeDelta -> Force -> Position -> RigidBody -> RigidBody
--updateMomentWithForce dt f p s = s &~ do
--    lmoment .= s ^. lmoment + dt *^ f
--    amoment .= s ^. amoment + dt * (f *. (p - s^.cog))


updateMomentWithTorque :: TimeDelta -> Torque -> RigidBody -> RigidBody
updateMomentWithTorque dt tau s = s & amoment %~ (+ dt * tau)


-- note : no position information
updateMoment :: TimeDelta -> (Force, Torque) -> RigidBody -> RigidBody
updateMoment dt (V2 fx fy, tau) s = s &~ do
        lmoment .= s ^. lmoment ^+^  (dt *^ (V2 fx fy))
        amoment .= s ^. amoment +  dt * tau


updatePosition :: TimeDelta -> RigidBody -> RigidBody
updatePosition dt s = s & cog %~ (+ (s ^. rmass * dt) *^ (s ^. lmoment))


updateAttitude :: TimeDelta -> RigidBody -> RigidBody
updateAttitude dt s = s & att %~ (+ s ^. rinertia * s ^. amoment * dt)


updateRigidBody :: TimeDelta -> RigidBody -> RigidBody
updateRigidBody dt = (updatePosition dt) . (updateAttitude dt)


---------------------------
-- SimpleConstraint
---------------------------

data LocalCoord = LocalCoord Int Position deriving (Ord, Eq, Show)


data Constraint = DistConstraint LocalCoord LocalCoord Float deriving (Ord, Eq, Show)


systemVelocity :: [RigidBody] -> [Float]
systemVelocity [] = []
systemVelocity (b:bs) = [x, y, b ^. rinertia * b^.amoment] ++ systemVelocity bs
    where
    V2 x y = b ^. rmass *^ b ^. lmoment


systemInvMass :: [RigidBody] -> [[Float]]
systemInvMass bs = f 0 bs
    where
    n = length bs
    f _ [] = []
    f i (b:bs) = v1:v2:v3: f (i+1) bs
        where
        v1 = ithVec (b^.rmass)    (3*i+0) n
        v2 = ithVec (b^.rmass)    (3*i+1) n
        v3 = ithVec (b^.rinertia) (3*i+2) n


decomposeSystemForce :: [Float] -> [(Force, Torque)]
decomposeSystemForce [] = []
decomposeSystemForce (fx:fy:fw:fs) = (V2 fx fy, fw):decomposeSystemForce fs


toSystemVector :: Int -> Int -> (V2 Float, Float) -> [Float]
toSystemVector 0 n (V2 x y, w) = [x,y,w] ++ replicate (3*(n-1)) 0
toSystemVector i n vs =  [0, 0, 0] ++ toSystemVector (i-1) (n-1) vs


-- calc j and v(push vector)
jv :: Constraint -> TimeDelta -> [RigidBody] -> ([Float], Float)
jv (DistConstraint (LocalCoord ia lra) (LocalCoord ib lrb) l) dt bs = (j, v)
    where
    beta = 0.5
    n = length bs
    ba = bs ^?! (ix ia)
    bb = bs ^?! (ix ib)
    xa = ba ^. cog
    xb = bb ^. cog
    ra = rotateVec (ba ^. att) lra
    rb = rotateVec (bb ^. att) lrb
    pa = xa ^+^ ra
    pb = xb ^+^ rb
    jav = pa ^-^ pb
    jaw = ra `crossZ` jav
    jbv = pb ^-^ pa
    jbw = rb `crossZ` jbv
    j = (toSystemVector ia n (jav, jaw)) ^+^ (toSystemVector ib n (jbv, jbw))
    v = (-1 / dt) * (norm (pa - pb) - l) * beta


calcJv :: [Constraint] -> TimeDelta -> [RigidBody] -> ([[Float]], [Float])
calcJv [] dt _ = ([], [])
calcJv (c:cs) dt bs = (j:js, v:vs)
    where
    (j, v) = jv c dt bs
    (js, vs) = calcJv cs dt bs


solveConstraintForce :: TimeDelta -> [[Float]] -> [[Float]] -> [Float] -> [Float] -> [Float]
solveConstraintForce dt matJ matInvM v v0 = (transpose matJ) !* lambda
    where
    matA = matJ !*! matInvM !*! transpose matJ
    b = (1.0 / dt) *^ (v  ^-^ matJ !* v0)
    lambda = gaussSeidel matA b 10


constraintForce :: TimeDelta -> [Constraint] -> [RigidBody] -> [(Force, Torque)]
constraintForce _ [] _ = []
constraintForce dt cs bs = decomposeSystemForce fc
    where
    (matJ, v) = calcJv cs dt bs
    matInvM = systemInvMass bs
    v0 = systemVelocity bs
    fc = solveConstraintForce dt matJ matInvM v v0


---------------------------
-- Simple Model Elements
---------------------------

disk :: Mass -> Inertia -> Position -> RigidBody
disk rm ri p = RigidBody rm ri p (0) (V2 0 0) 0


drawDisk :: Float -> RigidBody -> Picture
drawDisk r b = pictures $
        [ G.color black $ G.translate x y $ G.circle r
        , G.color black $ G.line [(x, y), (x1, y1)]
        ]
    where
    V2 x y = b ^. cog
    theta = b ^. att
    V2 x1 y1 = b ^. cog + r *^ (V2 (cos theta) (sin theta))


drawVector :: Position -> Force -> Picture
drawVector (V2 px py) (V2 fx fy) = G.color red $ G.line [(px, py), (px + s * fx, py + s * fy)]
    where
    s = 1


---------------------------
-- App
---------------------------

data MyCommand = Actuator Float

data App = App
    { _bodies :: [RigidBody]
    , _constraints :: [Constraint]
    , _time :: Time
    , _actuator :: Float
    , _force :: Force
    }


makeLenses ''App

offsetLR = 3
radiusLR = 1
radiusM  = 4

--
--initApp :: App
--initApp = App
--    { _bodies =
--        [ disk 1 1 (V2 (-1) 0)
--        , disk 1 1 (V2 1 0)
--        ]
--    , _constraints =
--        [ DistConstraint (LocalCoord 0 (V2 0 0)) (LocalCoord 1 (V2 0 0)) 2
--        ]
--    , _time = 0
--    , _actuator = 0
--    , _force = V2 0 0
--    }

initApp :: App
initApp = App
    { _bodies =
        [ disk 4 4 (V2 (-3) 0)
        , disk 4 4 (V2 3 0)
        , disk 1 1 (V2 0 0)
        ]
    , _constraints =
        [ DistConstraint (LocalCoord 0 (V2 0 0)) (LocalCoord 2 (V2 (-3) 0)) 0
        , DistConstraint (LocalCoord 1 (V2 0 0)) (LocalCoord 2 (V2 3 0)) 0
        ]
    , _time = 0
    , _actuator = 0
    , _force = V2 0 0
    }


scalePic :: Picture -> Picture
scalePic = scale scaleFactor scaleFactor
    where
    scaleFactor = 100 * 3 / 4

drawBody :: Int -> RigidBody -> Picture
--drawBody _ b = scalePic $ drawDisk 1 b
drawBody 0 b = scalePic $ drawDisk radiusLR b -- left
drawBody 1 b = scalePic $ drawDisk radiusLR b -- right
drawBody 2 b = scalePic $ drawDisk radiusM b  -- middle



---------------------------
-- main
---------------------------

defaultDisplay :: Display
defaultDisplay = InWindow "PhyToy" defaultScreenSize (100, 100)

defaultScreenSize = (1300, 700) -- [pxl]

drawApp :: App -> IO Picture
drawApp s = do
    let bs = s ^. bodies
    let msg = printf "%5.1f" $ s ^. time
    let txt = (translate (-550) (210) . scale 0.5 0.5 $ text $ msg)
    let f = scalePic $ drawVector (s ^?! bodies . ix 0 . cog) (s ^. force)
    ps <- forM [0..(length bs - 1)] $ \idx -> do
        return $ drawBody idx (bs !! idx)
    return $ pictures $ f:txt:ps



handleInput :: Event -> App -> IO App
handleInput (EventKey (Char 'q') _ _ _) s = die "exit"
handleInput (EventKey (Char 'w') Down _ _) s = return $ s & actuator .~ 0.1
handleInput (EventKey (Char 'w') Up _ _) s = return $ s & actuator .~ 0
handleInput (EventKey (Char 's') Down _ _) s = return $ s & actuator .~ (-0.1)
handleInput (EventKey (Char 's') Up _ _) s = return $ s & actuator .~ 0
handleInput ev s = return $ s & actuator .~ 0


stepApp :: Float -> App -> IO App
stepApp dt s = do
    let fe = s ^. actuator
    print $ (s^.time, s^.bodies)
    return $ s
        & bodies .~ foldr f (s^.bodies) (((-) 1) <$> [1..(length $ s^.bodies)])
        & bodies . traverse %~ (updateRigidBody dt)
        & bodies . (ix 0) %~ updateMomentWithTorque dt fe
        & bodies . (ix 1) %~ updateMomentWithTorque dt (fe)
        & bodies . (ix 2) %~ updateMomentWithTorque dt (-2*fe)
        & time %~ (+ dt)
        & force .~ fst (fc !! 0)
    where
        fc :: [(Force, Torque)]
        fc = constraintForce dt (s ^. constraints) (s ^. bodies)
        f :: Int -> [RigidBody] -> [RigidBody]
        f i bs = bs & ix i %~ updateMoment dt (fc !! i)


main :: IO ()
main = do
    --playIO defaultDisplay white 60 (initApp & bodies . ix 2 . amoment .~ 1) drawApp handleInput stepApp
    playIO defaultDisplay white 60 (initApp) drawApp handleInput stepApp

