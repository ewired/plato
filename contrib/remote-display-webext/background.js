// #region browser interactivity

let windowId;
browser.tabs.query({ active: true, currentWindow: true })
  .then((tabs) => {
    windowId = tabs[0].windowId;
  });

async function tabOffset(offset) {
  const tabs = await browser.tabs.query({ windowId });
  const currentTab = tabs.find((tab) => tab.active);
  const currentIndex = tabs.indexOf(currentTab);
  const newIndex = (currentIndex + offset + tabs.length) % tabs.length;
  await browser.tabs.update(tabs[newIndex].id, { active: true });
}

async function windowOffset(offset) {
  const windows = await browser.windows.getAll({ populate: true });
  const currentWindow = windows.find((window) => window.id === windowId);
  const currentIndex = windows.indexOf(currentWindow);
  const newIndex = (currentIndex + offset + windows.length) % windows.length;
  windowId = windows[newIndex].id;
  await new Promise((resolve) => setTimeout(resolve, 100));
}

async function moveTabToFirst() {
  const tabs = await browser.tabs.query({ windowId });
  const currentTab = tabs.find((tab) => tab.active);
  if (!currentTab) return;
  await browser.tabs.move(currentTab.id, { index: 0 });
}

async function moveTabToLast() {
  const tabs = await browser.tabs.query({ windowId });
  const currentTab = tabs.find((tab) => tab.active);
  if (!currentTab) return;
  await browser.tabs.move(currentTab.id, { index: -1 });
}

async function currentTab() {
  const [tab] = await browser.tabs.query({ windowId, active: true });
  return tab;
}

async function currentTabInfo() {
  const tabs = await browser.tabs.query({ windowId });
  const currentTab = tabs.find((tab) => tab.active);
  const currentTabIndex = tabs.indexOf(currentTab);
  const windows = await browser.windows.getAll({ populate: true });
  const currentWindow = windows.find((window) => window.id === windowId);
  const currentWindowIndex = windows.indexOf(currentWindow);
  const url = new URL(currentTab.url);
  return `W${currentWindowIndex + 1} T${currentTabIndex + 1
    }/${tabs.length} ${url.host}`;
}

async function scroll(pctX, pctY, verticalPct, horizontalPct = 0) {
  const { id } = await currentTab();
  await browser.tabs.executeScript(id, {
    code: `(() => {
      // Combined scrollability check function
      function isScrollable(element, direction) {
        if (!element) return false;
        
        const isVertical = direction === 'vertical';
        const hasOverflow = isVertical 
          ? Math.abs(element.scrollHeight - element.clientHeight) > 10
          : Math.abs(element.scrollWidth - element.clientWidth) > 10;
          
        if (!hasOverflow) return false;
        
        const style = window.getComputedStyle(element);
        const overflow = style.getPropertyValue(isVertical ? 'overflow-y' : 'overflow-x');
        const isScrollableStyle = ['auto', 'scroll', 'overlay'].includes(overflow);
        
        const isDefaultScrollable = element.tagName === 'BODY' ||
          element.tagName === 'HTML' ||
          element.tagName === 'DIV' && hasOverflow;
                                   
        return isScrollableStyle || isDefaultScrollable;
      }
      
      const elements = [...document.elementsFromPoint(
        window.innerWidth * ${pctX}, window.innerHeight * ${pctY}
      )];

      // Handle iframe elements
      if (elements[0]?.tagName === "IFRAME") {
        const iframeElements = elements[0].contentDocument?.elementsFromPoint(
          window.innerWidth * ${pctX}, window.innerHeight * ${pctY}
        );
        elements.unshift(...(iframeElements ?? []));
      }
      
      // Try scrolling elements separately for each direction
      let verticalScrolled = false;
      let horizontalScrolled = false;

      // Handle vertical scrolling
      if (${verticalPct} !== 0) {
        for (const el of elements) {
          if (isScrollable(el, 'vertical')) {
            const prevTop = el.scrollTop;
            el.scrollBy(0, window.innerHeight * ${verticalPct});
            if (el.scrollTop !== prevTop) {
              verticalScrolled = true;
              break; // Found and scrolled vertically, move on
            }
          }
        }
      }

      // Handle horizontal scrolling independently
      if (${horizontalPct} !== 0) {
        for (const el of elements) {
          if (isScrollable(el, 'horizontal')) {
            const prevLeft = el.scrollLeft;
            el.scrollBy(window.innerWidth * ${horizontalPct}, 0);
            if (el.scrollLeft !== prevLeft) {
              horizontalScrolled = true;
              break; // Found and scrolled horizontally, move on
            }
          }
        }
      }

      // Only return if both requested directions were handled
      if ((${verticalPct} === 0 || verticalScrolled) && (${horizontalPct} === 0 || horizontalScrolled)) {
        return;
      }
      
      // Fallback to window scrolling
      window.scrollBy(
        window.innerWidth * ${horizontalPct}, 
        window.innerHeight * ${verticalPct}
      );
    })()`,
  });
  browser.tabs.sendMessage(id, { type: 'WAIT_FOR_ANIMATIONS' });
}

async function zoomPage(addFactor) {
  const { id } = await currentTab();
  const factor = await browser.tabs.getZoom(id);
  let newFactor = factor + addFactor;
  if (newFactor < 0.3) newFactor = 0.3;
  if (newFactor > 5) newFactor = 5;
  await browser.tabs.setZoom(id, newFactor);
  return newFactor;
}

async function goForward() {
  const { id } = await currentTab();
  await browser.tabs.goForward(id);
}

async function goBack() {
  const { id } = await currentTab();
  await browser.tabs.goBack(id);
}

async function closeCurrentTab() {
  const { id } = await currentTab();
  await browser.tabs.remove(id);
}

async function reopenClosedTab() {
  const sessions = await browser.sessions.getRecentlyClosed();
  const lastSession = sessions
    .find((session) => session.tab && session.tab.windowId === windowId);
  if (!lastSession) return;
  await browser.sessions.restore(lastSession.tab.sessionId);
  await browser.tabs.update(lastSession.tab.id, { active: true });
  return new URL(lastSession.tab.url).host;
}

async function reloadCurrentTab() {
  const { id } = await currentTab();
  await browser.tabs.reload(id);
}

async function resizeViewport(width, height) {
  width = Math.round(width);
  height = Math.round(height);
  const window = await browser.windows.get(windowId);
  const tab = await currentTab();
  if (tab.width === width && tab.height === height) return;
  if (tab.width > width || tab.height > height) {
    await browser.windows.update(window.id, {
      width: width,
      height: height,
    });
  }
  const offsetWidth = window.width - tab.width;
  const offsetHeight = window.height - tab.height;
  await browser.windows.update(window.id, {
    width: width + offsetWidth,
    height: height + offsetHeight,
  });
  await new Promise((resolve) => setTimeout(resolve, 100));
}

async function openLinkUnderTap(pctX, pctY) {
  const { id, windowId } = await currentTab();
  const [url] = await browser.tabs.executeScript(id, {
    code:
      `[...document.elementsFromPoint(window.innerWidth * ${pctX}, window.innerHeight * ${pctY})]
       .find((e) => !!e.href)
       ?.href`,
  });
  if (!url) return;
  await browser.tabs.create({ url, windowId, openerTabId: id });
  await browser.tabs.update(id, { active: true });
  return new URL(url).host;
}

async function clickUnderTap(pctX, pctY) {
  const { id } = await currentTab();
  await browser.tabs.executeScript(id, {
    code: `(() => {
      const coordX = window.innerWidth * ${pctX};
      const coordY = window.innerHeight * ${pctY};
      const elements = [...document.elementsFromPoint(coordX, coordY)];
      if (elements[0]?.tagName === "IFRAME") {
        try {
          const iframeElements = elements[0].contentDocument?.elementsFromPoint(coordX, coordY);
          if (iframeElements?.length) {
            elements.unshift(...iframeElements);
          }
        } catch (e) {
          console.error("Error accessing iframe content:", e);
        }
      }
      function simulateMouseEvent(element, eventName) {
        if (!element) return false;
        const mouseEvent = new MouseEvent(eventName, {
          view: window,
          bubbles: true,
          cancelable: true,
          clientX: coordX,
          clientY: coordY,
          button: 0
        });
        
        const dispatched = element.dispatchEvent(mouseEvent);
        return dispatched;
      }
      function simulatePointerEvent(element, eventName) {
        if (!element) return false;
        
        // Create and dispatch the pointer event with coordinates
        const pointerEvent = new PointerEvent(eventName, {
          view: window,
          bubbles: true,
          cancelable: true,
          clientX: coordX,
          clientY: coordY,
          pointerId: 1,
          pointerType: 'mouse',
          isPrimary: true,
          pressure: 1,
          button: 0
        });
        
        const dispatched = element.dispatchEvent(pointerEvent);
        return dispatched;
      }
      for (const el of elements) {
        const isClickable = el.onclick ||
                           el.tagName === "A" ||
                           el.tagName === "BUTTON" ||
                           el.tagName === "INPUT" ||
                           el.tagName === "SELECT" ||
                           el.tagName === "TEXTAREA" ||
                           el.getAttribute("role") === "button" ||
                           window.getComputedStyle(el).cursor === "pointer";
        
        if (isClickable) {
          console.log("Clicking element:", el.tagName, el.className);
          simulatePointerEvent(el, "pointerdown");
          simulateMouseEvent(el, "mousedown");
          simulatePointerEvent(el, "pointerup");
          simulateMouseEvent(el, "mouseup");
          simulateMouseEvent(el, "click");
          return;
        }
      }
      
      for (const el of elements) {
        if (el.id && ['H1', 'H2', 'H3', 'H4', 'H5', 'H6'].includes(el.tagName)) {
          console.log("Header with ID found:", el.id);
          window.location.hash = el.id;
          return;
        }
      }
      
      if (elements.length > 0) {
        console.log("Fallback click on:", elements[0].tagName);
        simulatePointerEvent(elements[0], "pointerdown");
        simulateMouseEvent(elements[0], "mousedown");
        simulatePointerEvent(elements[0], "pointerup");
        simulateMouseEvent(elements[0], "mouseup");
        simulateMouseEvent(elements[0], "click");
      }
    })()`,
  });
  browser.tabs.sendMessage(id, { type: 'WAIT_FOR_ANIMATIONS' });
}

async function offsetContrastFilter(offset) {
  const { id } = await currentTab();
  const [newContrast] = await browser.tabs.executeScript(id, {
    code: `(() => {
      const el = document.documentElement;
      const match = el.style.filter?.match(/contrast\\((\\d+)%\\)/);
      const contrast = match ? parseInt(match[1]) : 100;
      const invert = el.style.filter?.includes("invert");
      const offsetContrast = contrast + ${offset};
      if (offsetContrast < 100) {
        el.style.filter = \`\${invert ? "" : "grayscale() invert() "}contrast(100%)\`;
      } else {
        el.style.filter = \`\${
          (offsetContrast === 100) && !invert ? "" : "grayscale() "}\${
          invert ? "invert() " : ""
        }contrast(\${offsetContrast}%)\`;
      }
      return el.style.filter.match(/contrast\\((\\d+)%\\)/)[1];
    })()`,
  });
  return newContrast;
}
// #endregion

// #region main loop

// Message listener for content script communication
browser.runtime.onMessage.addListener(async (message, sender) => {
  if (message.type === 'CAPTURE_SCREENSHOT') {
    console.log('Received screenshot request from content script');
    await sendImage();
    return true;
  }
});

let scaleFactor = 1;
let deviceWidth = 0;
let deviceHeight = 0;
let mq;

import mqtt from "https://esm.sh/mqtt@5.10.1";
import { encode, decode } from "https://esm.sh/cborg@4.2.4";
import { Encrypt0Message } from "https://esm.sh/@ldclabs/cose-ts@1.3.2/encrypt0";
import { hexToBytes } from "https://esm.sh/@ldclabs/cose-ts@1.3.2/utils";
import { Header } from "https://esm.sh/@ldclabs/cose-ts@1.3.2/header";
import { ChaCha20Poly1305Key } from "https://esm.sh/@ldclabs/cose-ts@1.3.2/chacha20poly1305";
import * as iana from "https://esm.sh/@ldclabs/cose-ts@1.3.2/iana";

import init, { png_to_display_update } from "./enc/crates_remote_display_video.js";
await init();

let topic = "remote-display-webext";
let key;

async function encodeCipher(msg) {
  if (!key) {
    throw new Error("no key");
  }
  const nonce = new Uint8Array(12);
  crypto.getRandomValues(nonce);
  return await new Encrypt0Message(
    msg instanceof Uint8Array ? msg : encode(msg),
    new Header().setParam(iana.HeaderParameterAlg, iana.AlgorithmChaCha20Poly1305),
    new Header().setParam(iana.HeaderParameterIV, nonce),
  ).toBytes(key);
}

async function send(msg) {
  if (mq?.connected) {
    await mq.publishAsync(`${topic}/device`, await encodeCipher(msg));
  }
}

async function decodeCipher(msg) {
  if (!key) {
    throw new Error("no key");
  }
  const emsg = await Encrypt0Message.fromBytes(key, new Uint8Array(msg));
  return decode(emsg.payload);
}

async function sendNotice(notice) {
  await send({ type: "notify", value: notice });
}

const sendImage = async (keyframe = false) => {
  if (!mq?.connected) return;
  if (!deviceWidth || !deviceHeight) {
    await send({ type: "updateSize" });
    console.log("cannot update image without size");
    return;
  }
  window.performance.mark("remote_display_capture");
  const { id } = await currentTab();
  const dataUrl = await browser.tabs.captureTab(id, {
    scale: scaleFactor,
  });
  const buf = await fetch(dataUrl)
    .then(a => a.arrayBuffer())
    .then(a => new Uint8Array(a));
  window.performance.mark("remote_display_capture_end");
  const captureMeasure = window.performance.measure("remote_display_capture", "remote_display_capture", "remote_display_capture_end");
  console.log(`captured PNG in ${captureMeasure.duration} ms`);

  try {
    window.performance.mark("remote_display_encode");
    const qoi = png_to_display_update(buf, deviceWidth, deviceHeight, keyframe);
    window.performance.mark("remote_display_encode_end");
    const encodeMeasure = window.performance.measure("remote_display_encode", "remote_display_encode", "remote_display_encode_end");
    if (qoi) {
      const size = qoi.length / 1024;
      console.log(`encoded display update of ${size.toFixed(2)} KB in ${encodeMeasure.duration} ms`);
      window.performance.mark("remote_display_send_image");
      await send(qoi);
    } else {
      console.error("Failed to encode display updates");
    }

    await new Promise((resolve) => {
      const updated = async (_, m) => {
        const msg = await decodeCipher(m);
        if (msg.type === "displayUpdated") {
          mq.off("message", updated);
          window.performance.mark("remote_display_send_image_end");
          const measure = window.performance.measure("remote_display_send_image", "remote_display_send_image", "remote_display_send_image_end");
          console.log(`resolved display update roundtrip in ${measure.duration} ms`);
          resolve();
        }
      }
      mq.on("message", updated);
    });
  } catch (e) {
    console.error(e);
  }
};

let timeout;
browser.tabs.onUpdated.addListener(async (_id, changeInfo, tab) => {
  if (!tab.active) return;
  if (changeInfo.status !== "complete") return;
  const myTab = await currentTab();
  if (tab.id !== myTab.id) return;
  if (timeout) {
    clearTimeout(timeout);
  }
  timeout = setTimeout(async () => {
    console.log("updating from tab load", changeInfo, tab);
    const info = await currentTabInfo();
    await sendNotice(info);
    await sendImage(true);
  }, 1000);
});

async function onMessage(msg) {
  console.log("message", msg);
  switch (msg.type) {
    case "size": {
      const { width, height } = msg.value;
      deviceWidth = width;
      deviceHeight = height;
      await sendImage(true);
      break;
    }
    case "swipe": {
      const { dir, start, end } = msg.value;
      const dx = end.x - start.x;
      const dy = end.y - start.y;
      await scroll(
        start.x / deviceWidth,
        start.y / deviceHeight,
        ["north", "south"].includes(dir) ? -(dy / deviceHeight) : 0,
        ["east", "west"].includes(dir) ? -(dx / deviceWidth) : 0
      );
      await sendImage();
      break;
    }
    case "slantedSwipe": {
      const { start, end } = msg.value;
      const dx = end.x - start.x;
      const dy = end.y - start.y;

      const horizontalPct = -(dx / deviceWidth);
      const verticalPct = -(dy / deviceHeight);

      await scroll(
        start.x / deviceWidth,
        start.y / deviceHeight,
        verticalPct,
        horizontalPct
      );
      await sendImage();
      break;
    }
    case "button": {
      const { button, status } = msg.value;
      if (!["released", "repeated"].includes(status)) break;
      // scroll half pages
      switch (button) {
        case "forward": {
          await scroll(0.5, 0.5, 0.5, 0);
          break;
        }
        case "backward": {
          await scroll(0.5, 0.5, -0.5, 0);
          break;
        }
      }
      if (status === "released") await sendImage();
      break;
    }
    case "arrow": {
      const { dir } = msg.value;
      switch (dir) {
        case "east":
          await goForward();
          break;
        case "west":
          await goBack();
          break;
        case "north": {
          const info = await currentTabInfo();
          await sendNotice(`closing ${info}`);
          await closeCurrentTab();
          const newInfo = await currentTabInfo();
          await sendNotice(newInfo);
          await sendImage();
          break;
        }
      }
      break;
    }
    case "multiSwipe": {
      const { dir, starts, ends } = msg.value;
      switch (dir) {
        case "east":
        case "west": {
          const offset = dir === "east" ? -1 : 1;
          const avgStartX = starts.reduce((sum, point) => sum + point.x, 0) / starts.length;
          const avgEndX = ends.reduce((sum, point) => sum + point.x, 0) / ends.length;
          const dx = avgEndX - avgStartX;
          // change windows if swipe covers more than half the screen
          await (Math.abs(dx) > deviceHeight / 2
            ? windowOffset(offset)
            : tabOffset(offset));
          const info = await currentTabInfo();
          await sendNotice(info);
          await sendImage(true);
          break;
        }
        case "north":
        case "south": {
          const tabs = await browser.tabs.query({ windowId });
          const targetIndex = dir === "south" ? 0 : tabs.length - 1;
          await browser.tabs.update(tabs[targetIndex].id, { active: true });
          const info = await currentTabInfo();
          await sendNotice(info);
          await sendImage(true);
          break;
        }
      }
      break;
    }
    case "multiArrow": {
      const { dir } = msg.value;
      switch (dir) {
        case "east":
          await moveTabToLast();
          break;
        case "west":
          await moveTabToFirst();
          break;
      }
      const info = await currentTabInfo();
      await sendNotice(info);
      await sendImage(true);
      break;
    }
    case "pinch":
    case "spread": {
      const newFactor = await zoomPage(msg.type === "spread" ? 0.1 : -0.1);
      await sendNotice(`zoom ${(100.0 * newFactor).toFixed(0)}%`);
      await sendImage();
      break;
    }
    case "rotate":
      if (Math.abs(msg.value.angle) < 20) break;
      if (msg.value.angle > 0) {
        const restoredTab = await reopenClosedTab();
        if (restoredTab) await sendNotice(`restored ${restoredTab}`);
        else await sendNotice("no tab restored");
        await sendImage();
      } else {
        await reloadCurrentTab();
      }
      break;
    case "holdFingerShort":
      await resizeViewport(deviceWidth / scaleFactor, deviceHeight / scaleFactor);
      await sendImage(true);
      await send({ type: "refreshDisplay" });
      break;
    case "holdFingerLong": {
      const [{ x, y }] = msg.value;
      const tab = await openLinkUnderTap(x / deviceWidth, y / deviceHeight);
      if (tab) await sendNotice(`${tab} opened`);
      else await sendNotice("no link under finger");
      break;
    }
    case "tap": {
      const { x, y } = msg.value;
      await clickUnderTap(x / deviceWidth, y / deviceHeight);
      await sendImage();
      break;
    }
    case "corner": {
      const { dir } = msg.value;
      switch (dir) {
        case "southWest":
        case "southEast":
          {
            const newContrast = await offsetContrastFilter(
              dir === "southWest" ? -25 : 25,
            );
            await sendNotice(`contrast ${newContrast}%`);
            await sendImage(true);
          }
          break;
        case "northWest":
        case "northEast": {
          const newScaleFactor = scaleFactor + (dir === "northWest" ? -0.1 : 0.1);
          if (newScaleFactor < 0.1 || newScaleFactor > 2) break;
          scaleFactor = newScaleFactor;
          await sendNotice(`scale ${Math.round(scaleFactor * 100)}%`);
          await resizeViewport(deviceWidth / scaleFactor, deviceHeight / scaleFactor);
          await sendImage(true);
        }
      }
      break;
    }
  }
}

const defaultConfig = {
  wsUrl: "wss://broker.hivemq.com:8884/mqtt",
  topic: "remote-display-webext",
  enabled: false,
  key: "",
};

async function getConfig() {
  const result = await browser.storage.local.get(["wsUrl", "topic", "enabled", "key"]);
  topic = result.topic || defaultConfig.topic;
  const hexKey = result.key || defaultConfig.key;
  key = ChaCha20Poly1305Key.fromSecret(hexToBytes(hexKey));
  await browser.storage.local.set({ ...defaultConfig, ...result });
  return { ...defaultConfig, ...result };
}


async function refreshConnection(config) {
  if (config.enabled && !mq?.connected) {
    mq = mqtt.connect(config.wsUrl, {
      will: {
        topic: `${config.topic}/device`,
        payload: await encodeCipher({ type: "notify", value: "Browser disconnected" }),
        qos: 0,
      }
    });
    mq.subscribe(`${config.topic}/browser`);
    mq.on("message", (_topic, message) => decodeCipher(message).then(onMessage));
    mq.on("connect", () => {
      sendNotice("Browser connected")
        .then(() => send({ type: "updateSize" }))
        .then(() => { console.log("connected"); })
    });
    mq.on("disconnect", () => {
      console.log("disconnected");
    });
  } else if (!config.enabled && mq?.connected) {
    mq.end();
  }
}

getConfig().then(refreshConnection).catch(console.error);

browser.storage.local.onChanged.addListener((changes) => {
  if (
    (
      ("wsUrl" in changes && changes.wsUrl.newValue !== changes.wsUrl.oldValue)
      || ("topic" in changes && changes.topic.newValue !== changes.topic.oldValue)
    ) &&
    mq?.connected
  ) {
    mq.end();
    getConfig().then(refreshConnection).catch(console.error);
  }
  if ("enabled" in changes && changes.enabled.newValue !== changes.enabled.oldValue) {
    getConfig().then(refreshConnection).catch(console.error);
  }
});

// #endregion
