{-# OPTIONS_GHC -Wno-unused-imports #-}
module Main where

import qualified Copilot.Language as C
import qualified Copilot.Compile.C99 as C99
import qualified Copilot.Core as Core
import qualified Copilot.Core.Type as Ty
import           Copilot.Language.Reify (reify)

-- Import your monitor module:
import CuriosityMonitor (spec)

main :: IO ()
main = do
  -- Change the prefix (file name) and output dir here if desired:
  let outName = "copilot"              -- produces copilot.c / copilot.h
  let outDir  = "copilot"              -- directory where files are written
  let cs = (C99.mkDefaultCSettings
             { C99.cSettingsOutputDirectory = outDir
             -- , C99.cSettingsStepFunctionName = "step"
             })

  coreSpec <- reify spec  -- :: Core.Spec

  C99.compileWith cs outName coreSpec
